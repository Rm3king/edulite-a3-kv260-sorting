#!/usr/bin/env python3
"""Read-only submission/runtime file checks. Does not connect to cameras, SSH or motors."""
import argparse
import json
from pathlib import Path
import sys


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--app-root',type=Path,help='Configured aipc_app runtime directory; omit to check the submission only')
    a=p.parse_args()
    package=next((r for r in Path(__file__).resolve().parents if (r/'build/source_bundle.json').is_file()),None)
    if package is None:raise SystemExit('Run this file from the complete submission package')
    checks=[]
    def check(name,path):
        path=Path(path);checks.append(dict(item=name,path=str(path),present=path.is_file()));return path.is_file()
    check('console source',package/'src/host/app/workcell/console.py')
    check('robot URDF',package/'board/arm_runtime/model/el_a3_legacy.urdf')
    check('GPU detector weights',package/'data/model/parts_v2_best.pt')
    check('NPU detector model',package/'data/model/parts_v2_a8w8_736x1280.onnx')
    for name in ('camera_to_base.json','workplane.json','route_geometry.xml','servo_config.json'):
        check('reference '+name,package/'data/workcell_reference'/name)
    if a.app_root:
        app=a.app_root.expanduser().resolve();config=app/'integration/config/live_workflow.json'
        if check('runtime workflow',config):
            settings=json.loads(config.read_text())
            for name in ('calibration','workplane','geometry_xml','heights','ssh_helper'):
                value=settings.get(name)
                if not value:checks.append(dict(item=name,path=None,present=False));continue
                check(name,Path(value).expanduser())
            for name,entry in [('runtime_root','joint_trajectory.py'),('can_root','canctl.py')]:
                check(name,Path(settings.get(name,''))/entry)
        check('console entry',app/'integration/workcell/console.py')
        check('wrist local calibration',app/'reports/wrist_online_20261025_160952/servo_config.json')
        check('SDK launcher',Path.home()/'.local/share/aipc-dev/aipc.sh')
    result={'scope':'file presence only; no device/network access; no execution authorization','package':str(package),'checks':checks,'status':'PASS' if all(x['present'] for x in checks) else 'NEEDS_CONFIGURATION'}
    print(json.dumps(result,ensure_ascii=False,indent=2))
    return 0 if result['status']=='PASS' else 2


if __name__=='__main__':sys.exit(main())
