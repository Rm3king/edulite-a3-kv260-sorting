#!/usr/bin/python3
"""Evaluate the independent instruction policy on preserved raw model outputs."""
import argparse,json
from pathlib import Path
from screwcell.tasks import validate_instruction,select_objects
p=argparse.ArgumentParser();p.add_argument('--vlm',required=True);p.add_argument('--scene',required=True);p.add_argument('--output',required=True);a=p.parse_args()
vlm=json.loads(Path(a.vlm).read_text());scene=json.loads(Path(a.scene).read_text());rows=[]
for row in vlm['rows']:
    result=dict(instruction=row['instruction'],raw_model_matches_expected=row['matches_development_expectation'])
    try:
        value=validate_instruction(row['raw'],row['instruction']);result['validated_intent']=value
        try:result.update(decision='ACCEPTED',objects=[x['object_id'] for x in select_objects(value,scene)])
        except ValueError as exc:result.update(decision='REJECTED',reason=str(exc))
    except ValueError as exc:result.update(decision='REJECTED_OR_CLARIFY',reason=str(exc))
    rows.append(result)
report=dict(status='RAW_FAILURES_RETAINED_POLICY_REVIEW',raw_model_correct=vlm['passed'],development_cases=vlm['total'],rows=rows,
            scope='Policy designed using these development cases; not a held-out evaluation; no motion')
Path(a.output).write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print(json.dumps(report,ensure_ascii=False,indent=2))
