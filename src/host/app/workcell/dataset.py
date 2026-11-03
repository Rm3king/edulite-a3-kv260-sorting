"""Deterministic synthetic instruction curriculum, split by wording families.

These labels are engineering-authored, not human-reviewed real user accuracy.
Evaluation output remains separate from the training artifacts.
"""
import argparse
import hashlib
import json
from pathlib import Path
from .tasks import result, SYSTEM

def step(c='screw',q=2,r='all',d='B'):
    return dict(part_class=c,quantity=q,region=r,destination=d)

def make(split, expanded=False):
    # Whole expression families belong to exactly one split.
    forms={
      'train':['把{region}{number}{part}放到{dest}盒。','请把{region}{number}{part}移到{dest}盒。','将{region}{number}{part}送到{dest}盒。'],
      'validation':['麻烦帮我把{region}{number}{part}搬进{dest}盒。'],
      'test':['这次需要把{region}{number}{part}收进{dest}盒。','请完成以下操作：{region}{number}{part}转移至{dest}盒。']}
    rows=[]
    def add(text,value,family,context=None):
        identity=split+'\0'+text
        if context and expanded and split=='train':identity+='\0'+json.dumps(context,sort_keys=True,ensure_ascii=False)
        key=hashlib.sha256(identity.encode()).hexdigest()[:16]
        rows.append(dict(id=key,split=split,family=family,instruction=text,context=context or {},expected=value,
                         source='ENGINEERING_AUTHORED_SYNTHETIC',human_reviewed=False))
    for f,form in enumerate(forms[split]):
        for cls,name in [('screw','螺丝'),('nut','螺母')]:
            for r,region in [('all',''),('left','左侧'),('right','右侧')]:
                for q,number in [(1,'一颗'),(2,'两颗'),(3,'三颗'),('all','全部')]:
                    for d in ['A','B']:
                        text=form.format(region=region,number=number,part=name,dest=d)
                        add(text,result(steps=[step(cls,q,r,d)]),'positive_'+str(f))
    negatives={
     'train':['把螺丝放到B盒。','拿三颗放到A盒。','把两颗螺丝放C盒。','把所有红色螺丝放B盒。','收拾一下桌面。'],
     'validation':['请将螺母移到A盒。','送两个东西进B盒。','把一颗螺母移到D盒。','把短螺丝放B盒。'],
     'test':['麻烦转移螺丝到B盒。','把两颗螺栓收进C盒。','请处理那几个。','将所有蓝色螺母收进A盒。','把一颗垫圈放A盒。']}
    for text in negatives[split]:add(text,result('clarify',question='请明确支持的类别、数量或目的盒。'),'clarify')
    variants={
     'train':('暂停','继续','先把两颗螺丝放B盒，然后把一颗螺母放A盒。','剩下的放B盒。','把所有螺丝放B盒，螺母不要动。'),
     'validation':('先停一下','恢复任务','先送两颗螺丝进B盒，再送一颗螺母进A盒。','其余螺丝放B盒。','请把全部螺丝移到B盒，留下螺母。'),
     'test':('暂停当前操作','接着执行未完成任务','分两步：两颗螺丝转移至B盒，接着一颗螺母转移至A盒。','剩余的也放B盒。','将螺丝全部收进B盒，保留原处的螺母。')}
    x=variants[split]
    add(x[0],result('pause'),'pause');add(x[1],result('resume'),'resume')
    add(x[2],result(steps=[step(),step('nut',1,'all','A')]),'sequence')
    add(x[3],result(steps=[step(q='all')]),'context',dict(last_part_class='screw'))
    add(x[4],result(steps=[step(q='all')],exclude=['nut']),'exclude')
    if expanded and split=='train':
        # v2 development expansion. Validation/test strings and labels stay frozen.
        for cls,name,other,other_name in [('screw','螺丝','nut','螺母'),('nut','螺母','screw','螺丝')]:
            for q,number in [(1,'一颗'),(2,'两颗'),(3,'三颗'),('all','全部')]:
                for r,region in [('all',''),('left','左侧'),('right','右侧')]:
                    for d in ['A','B']:
                        for suffix in [f'，留下{other_name}。',f'，别移动{other_name}。']:
                            add(f'将{region}{number}{name}送进{d}盒'+suffix,
                                result(steps=[step(cls,q,r,d)],exclude=[other]),'exclude')
            for d in ['A','B']:
                add(f'其余的送进{d}盒。类别沿用上次。',result(steps=[step(cls,'all','all',d)]),'context',dict(last_part_class=cls))
                for q in [1,2,3]:
                    add(f'首先送{q}颗{name}到{d}盒，随后送1颗{other_name}到'+('B' if d=='A' else 'A')+'盒。',
                        result(steps=[step(cls,q,'all',d),step(other,1,'all','B' if d=='A' else 'A')]),'sequence')
        for text in ['先别动','停止当前任务','暂停一下执行','请继续','恢复之前的操作','接着干']:
            add(text,result('pause' if text in ['先别动','停止当前任务','暂停一下执行'] else 'resume'),'control')
        for text in ['将一颗螺丝送进盒子。','拿几个螺母放A盒。','右侧的送进B盒。','将所有长螺钉送进A盒。','把两颗螺母放到Z盒。','其余的放A盒。']:
            add(text,result('clarify',question='请补充类别、数量或支持的目的盒；当前无法确定。'),'clarify')
    return rows

def prompt(row):
    return 'context='+json.dumps(row.get('context',{}),ensure_ascii=False,separators=(',',':'))+'\n指令：'+row['instruction']

def write(root, expanded=False):
    root=Path(root);root.mkdir(parents=True,exist_ok=True)
    allrows=[];manifest=dict(schema='workcell.curriculum.v1',label_scope='Synthetic, program-authored; human review and real scene evaluation pending',splits={})
    for split in ['train','validation','test']:
        rows=make(split,expanded);allrows+=rows
        raw=''.join(json.dumps(r,ensure_ascii=False,separators=(',',':'))+'\n' for r in rows).encode()
        (root/(split+'.jsonl')).write_bytes(raw)
        manifest['splits'][split]=dict(rows=len(rows),sha256=hashlib.sha256(raw).hexdigest())
    assert len({prompt(r) for r in allrows})==len(allrows)
    manifest['system_prompt_sha256']=hashlib.sha256(SYSTEM.encode()).hexdigest()
    (root/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    return manifest

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('output');p.add_argument('--expanded',action='store_true');a=p.parse_args();print(json.dumps(write(a.output,a.expanded),indent=2))
