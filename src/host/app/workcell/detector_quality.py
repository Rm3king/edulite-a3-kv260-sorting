"""Compare frozen float/Quark/NPU output geometry on old saved frames.

Agreement is against the float model, not independently labelled accuracy.
Raw outputs are retained locally; images/models are never changed.
"""
import argparse
import json
import math
import time
from pathlib import Path
import numpy as np
from .quark_pipeline import tensor


def collect(a):
    import onnxruntime as ort
    app=Path(a.app_root);out=Path(a.output);out.mkdir(parents=True,exist_ok=False)
    opts=ort.SessionOptions();opts.intra_op_num_threads=4;opts.log_severity_level=3
    float_path=app/'reports/amd_task_benchmark_20261020/detector.onnx'
    quant=Path(a.quantized_model) if a.quantized_model else app/'reports/quark_detector_20261020/detector_a8w8.onnx'
    options={'cacheDir':str(out/'compiler_cache'),'cacheKey':'screwseg_quality','config_file':a.config}
    sessions={'float_cpu':ort.InferenceSession(str(float_path),opts,providers=['CPUExecutionProvider']),
              'quant_cpu':ort.InferenceSession(str(quant),opts,providers=['CPUExecutionProvider']),
              'quant_npu':ort.InferenceSession(str(quant),opts,providers=[('VitisAIExecutionProvider',options),'CPUExecutionProvider'])}
    data=app/'data/screw_nut_test_v1';manifest=json.loads((data/'manifest.json').read_text());records=[]
    for i,entry in enumerate(manifest['images']):
        feed=tensor(data/entry['file']);values={}
        for name,s in sessions.items():
            heads=s.run(None,{s.get_inputs()[0].name:feed})
            for j,h in enumerate(heads):values[name+'_'+str(j)]=h
        np.savez_compressed(out/(str(i)+'.npz'),**values);records.append(dict(index=i,**entry))
    (out/'inputs.json').write_text(json.dumps(records,indent=2)+'\n')
    print(json.dumps(dict(status='COLLECTED_FLOAT_QUANT_NPU_HEADS',frames=len(records))),flush=True)


def iou(a,b):
    lo=np.maximum(a[:2],b[:2]);hi=np.minimum(a[2:],b[2:]);inter=np.maximum(0,hi-lo).prod()
    return float(inter/max(np.maximum(0,a[2:]-a[:2]).prod()+np.maximum(0,b[2:]-b[:2]).prod()-inter,1e-10))


def decode(heads,width,height):
    import cv2
    prediction,proto=heads;pred=prediction[0].T;cls=np.argmax(pred[:,4:6],axis=1);conf=np.max(pred[:,4:6],axis=1)
    ids=np.flatnonzero(conf>.25);ids=ids[np.argsort(-conf[ids])];selected=[];boxes=pred[:,:4].copy()
    boxes[:,:2]-=boxes[:,2:]/2;boxes[:,2:]+=boxes[:,:2]
    while len(ids) and len(selected)<300:
        k=int(ids[0]);selected.append(k);ids=np.array([j for j in ids[1:] if cls[j]!=cls[k] or iou(boxes[j],boxes[k])<=.7],dtype=int)
    results=[];ratio=640/max(width,height);nw,nh=round(width*ratio),round(height*ratio);padx,pady=(640-nw)//2,(640-nh)//2
    yy,xx=np.indices((640,640))
    for k in selected:
        logits=pred[k,6:]@proto[0].reshape(32,-1);mask=cv2.resize(logits.reshape(160,160),(640,640),interpolation=cv2.INTER_LINEAR)>0
        b=boxes[k];mask&=(xx>=b[0])&(xx<b[2])&(yy>=b[1])&(yy<b[3])
        mask=cv2.resize(mask[pady:pady+nh,padx:padx+nw].astype(np.uint8),(width,height),interpolation=cv2.INTER_NEAREST)>0
        box=(b-[padx,pady,padx,pady])/ratio
        results.append(dict(part_class='screw' if cls[k]==0 else 'nut',score=float(conf[k]),box=box,mask=mask))
    return results


def compare(reference,other,shaft):
    unused=set(range(len(other)));matches=[];missed=[]
    for i,r in enumerate(reference):
        choices=[(iou(r['box'],other[j]['box']),j) for j in unused if r['part_class']==other[j]['part_class']]
        overlap,j=max(choices,default=(0,None))
        if overlap<.5:missed.append(i);continue
        unused.remove(j);o=other[j];union=np.logical_or(r['mask'],o['mask']).sum();agreement=float(np.logical_and(r['mask'],o['mask']).sum()/max(union,1))
        row=dict(reference=i,candidate=j,part_class=r['part_class'],box_iou=overlap,mask_iou=agreement,score_change=o['score']-r['score'])
        if r['part_class']=='screw':
            try:
                x,y=shaft(r['mask']),shaft(o['mask']);diff=abs(x['axis_angle_image_deg_mod180']-y['axis_angle_image_deg_mod180'])%180
                row.update(grasp_center_delta_px=float(np.linalg.norm(np.asarray(x['center_uv_px'])-y['center_uv_px'])),axis_delta_deg=min(diff,180-diff))
            except ValueError as e:row['geometry_error']=str(e)
        matches.append(row)
    return dict(reference_count=len(reference),candidate_count=len(other),missed_reference=missed,unmatched_candidate=sorted(unused),matches=matches)


def analyze(a):
    import sys
    from PIL import Image,ImageDraw
    app=Path(a.app_root);sys.path.insert(0,str(app/'src'));from preview_part_grasps import shaft_candidate
    out=Path(a.output);records=json.loads((out/'inputs.json').read_text());rows=[]
    for e in records:
        with np.load(out/(str(e['index'])+'.npz'),allow_pickle=False) as heads:
            decoded={name:decode([heads[name+'_0'],heads[name+'_1']],e['width'],e['height']) for name in ['float_cpu','quant_cpu','quant_npu']}
        rows.append(dict(scene_id=e['scene_id'],quantization=compare(decoded['float_cpu'],decoded['quant_cpu'],shaft_candidate),
                         deployed=compare(decoded['float_cpu'],decoded['quant_npu'],shaft_candidate),npu_vs_quant_cpu=compare(decoded['quant_cpu'],decoded['quant_npu'],shaft_candidate)))
        if e['index']==0:
            panels=[]
            for name,objects in decoded.items():
                img=Image.open(app/'data/screw_nut_test_v1'/e['file']).convert('RGB');draw=ImageDraw.Draw(img);draw.text((12,12),name,fill='yellow')
                for o in objects:
                    draw.rectangle(o['box'].tolist(),outline='orange',width=2)
                    if o['part_class']=='screw':
                        try:
                            c=shaft_candidate(o['mask']);uv=c['center_uv_px'];draw.line([tuple(x) for x in c['axis_endpoints_uv_px']],fill='yellow',width=2);draw.ellipse((uv[0]-3,uv[1]-3,uv[0]+3,uv[1]+3),fill='red')
                        except ValueError:pass
                panels.append(img)
            canvas=Image.new('RGB',(e['width']*3,e['height']));[canvas.paste(im,(i*e['width'],0)) for i,im in enumerate(panels)];canvas.save(out/'comparison.png')
    report=dict(status='MEASURED_MODEL_AGREEMENT_NOT_ACCURACY',frames=rows,real_calibration='WAITING',production_backend='ROCM_FP32_UNTIL_EXTERNAL_GRASP_VALIDATION',motor_commands_sent=0)
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['collect','analyze']);p.add_argument('--app-root',required=True);p.add_argument('--output',required=True);p.add_argument('--config');p.add_argument('--quantized-model');a=p.parse_args();(collect if a.mode=='collect' else analyze)(a)
