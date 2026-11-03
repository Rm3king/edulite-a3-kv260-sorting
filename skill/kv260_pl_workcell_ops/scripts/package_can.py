#!/usr/bin/env python3
from pathlib import Path
import os,sys
for parent in Path(__file__).resolve().parents:
    for relative in ('build/scripts/rebuild.py','fpga/build/scripts/rebuild.py'):
        entry=parent/relative
        if entry.is_file():
            os.execv(sys.executable,[sys.executable,str(entry),'package',*sys.argv[1:]])
raise SystemExit('Keep this script inside the complete submission package')
