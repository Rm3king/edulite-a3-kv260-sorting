#!/usr/bin/python3
"""Parse a new Chinese task using installed local Qwen; never connects to a robot."""
import argparse,json
from pathlib import Path
from screwcell.model import LocalTaskModel
p=argparse.ArgumentParser();p.add_argument('--app-root',type=Path,required=True);p.add_argument('--scene',type=Path,required=True);p.add_argument('--instruction',required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
if a.output.exists():raise SystemExit('Use a new output path')
scene=json.loads(a.scene.read_text());model=LocalTaskModel(a.app_root);result=model.infer(a.instruction,scene)
a.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');print(json.dumps(result,ensure_ascii=False,indent=2))
