"""Actual saved-adapter reload, new text/image task and protocol-sim demonstration.

Real RGB-D images remain recorded data. Simulation binds its own synthetic scene;
the real image never supplies a fictitious calibrated physical robot target.
"""
import argparse
import json
import os
import subprocess
import sys
from pathlib import Path
from .tasks import validate,semantic_key,result
from .dataset import step


def run(a):
    app=Path(a.app_root);out=Path(a.output);out.mkdir(parents=True,exist_ok=False);records=[]
    stderr=(out/'worker.stderr.log').open('w')
    p=subprocess.Popen([sys.executable,'-u','-m','workcell.model_server','--app-root',str(app),'--adapter',a.adapter],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=stderr,text=True,bufsize=1)
    def read_event():
        while True:
            line=p.stdout.readline()
            if not line:raise RuntimeError('MODEL_WORKER_EXIT_'+str(p.poll()))
            try:v=json.loads(line)
            except ValueError:continue
            records.append(v);(out/'worker.json').write_text(json.dumps(records,ensure_ascii=False,indent=2)+'\n')
            if v.get('event') in ('MODEL_READY','MODEL_RESULT','MODEL_ERROR'):return v
    def request(v):p.stdin.write(json.dumps(v,ensure_ascii=False)+'\n');p.stdin.flush();return read_event()
    try:
        ready=read_event();assert ready['event']=='MODEL_READY'
        if a.reload_report:
            old=json.loads(Path(a.reload_report).read_text())[0]
            rows=[json.loads(x) for x in (app/'data/task_curriculum_v2/test.jsonl').read_text().splitlines()]
            row=next(x for x in rows if x['id']==old['id'])
            reloaded=request(dict(request_id='reload_equivalence',instruction=row['instruction'],context=row['context']))
            assert reloaded.get('raw')==old['raw'],'SAVED_ADAPTER_RELOAD_GENERATION_CHANGED'
        instruction='把一颗螺丝放到B盒，螺母不要动。';expected=result(steps=[step(q=1)],exclude=['nut'])
        text=request(dict(request_id='new_composition_text',instruction=instruction,context={}))
        assert text['event']=='MODEL_RESULT',text
        assert semantic_key(text['intent'])==semantic_key(expected),'NEW_COMPOSITION_SEMANTIC_MISMATCH'
        (out/'actual_model_plan.json').write_text(json.dumps(text['intent'],ensure_ascii=False,indent=2)+'\n')
        visual=None
        if a.scene:
            scene=json.loads(Path(a.scene).read_text());visual=request(dict(request_id='recorded_image_composition',instruction=instruction,context={},image_file=scene['image_file'],scene=scene))
            # Preserve image failures separately; they do not authorize motion.
            (out/'visual_result.json').write_text(json.dumps(visual,ensure_ascii=False,indent=2)+'\n')
        report=dict(status='PASS_RELOADED_ADAPTER_TEXT_TOOL_DEMO',reload_equivalence=bool(a.reload_report),
            new_composition_semantic_correct=True,visual_contract_valid=bool(visual and visual['event']=='MODEL_RESULT'),
            visual_semantic_correct=bool(visual and visual['event']=='MODEL_RESULT' and semantic_key(visual['intent'])==semantic_key(expected)),
            scope='Recorded-image task understanding and synthetic scene execution are separate; no real calibrated target',motor_commands_sent=0)
        (out/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report),flush=True)
    except Exception as e:
        (out/'failure.json').write_text(json.dumps(dict(status='FAILED_RETAINED',error=repr(e),motor_commands_sent=0),ensure_ascii=False,indent=2)+'\n');raise
    finally:
        if p.poll() is None:
            try:p.stdin.write('{"op":"shutdown"}\n');p.stdin.flush();p.wait(timeout=15)
            except (BrokenPipeError,subprocess.TimeoutExpired):p.terminate();p.wait(timeout=10)
        stderr.close()
    if a.simulate:
        with (out/'simulation.log').open('w') as log:
            subprocess.run([str(Path.home()/'.local/share/aipc-dev/aipc.sh'),'sim','python','-u','-m','workcell.demo',
                '--urdf',str(app/'integration/workcell/assets/el_a3_legacy.urdf'),'--output',str(out/'simulation'),
                '--plan',str(out/'actual_model_plan.json'),'--test-pause'],cwd=app/'integration',stdout=log,stderr=subprocess.STDOUT,
                env=dict(os.environ,MUJOCO_GL='egl',PYOPENGL_PLATFORM='egl'),check=True,timeout=240)
        report.update(status='PASS_ACTUAL_QWEN_TO_TCP_TO_CONTACT_SIMULATION',simulation=json.loads((out/'simulation/report.json').read_text())['status'])
        (out/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--app-root',required=True);p.add_argument('--adapter',required=True);p.add_argument('--output',required=True);p.add_argument('--reload-report');p.add_argument('--scene');p.add_argument('--simulate',action='store_true');run(p.parse_args())
