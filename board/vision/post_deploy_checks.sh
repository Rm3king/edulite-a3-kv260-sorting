#!/bin/bash
# Board checks after loading the correction-channel bitstream. Arm motors are powered off; nothing is enabled.
cd ~/kv260_can_deploy
out=~/post_deploy_$(date +%Y%m%d_%H%M%S); mkdir -p $out
echo "== status"; python3 canctl.py status > $out/status.json 2>&1; head -c 400 $out/status.json; echo
echo "== board_validate"; python3 board_validate.py > $out/board_validate.json 2>&1; echo rc=$?; tail -c 300 $out/board_validate.json; echo
echo "== tracking monitor"; python3 tracking_monitor.py validate > $out/tracking.json 2>&1; echo rc=$?; grep -o '"status": "[A-Z_]*"' $out/tracking.json | head -2
echo "== vision kernel"; python3 -c "from PIL import Image;import numpy as np;np.asarray(Image.open('/home/ubuntu/vl_replay/live_task_20261029_182140_a497b9_iteration_01/iteration_00/frame_0.png').convert('L'),np.uint8).tofile('/tmp/pd_frame.raw')"
python3 vision_pre.py selftest /tmp/pd_frame.raw > $out/vision_selftest.txt 2>&1; echo rc=$?; tail -3 $out/vision_selftest.txt
python3 vision_pre.py bench /tmp/pd_frame.raw > $out/vision_bench.txt 2>&1; echo rc=$?; tail -4 $out/vision_bench.txt
echo "== correction channel"; python3 board_correction_validate.py > $out/correction.json 2>&1; echo rc=$?; grep -E '"status"|reset_on_stop|"ok": false' $out/correction.json | head
echo "== four-part locator replay from the deployed bundle (46 frames)"; (cd ~/vl_replay4 && timeout 600 python3 ~/kv260_can_deploy/wrist_vision_loop.py replay $(ls -d live_task_*) > $out/replay4.jsonl 2>&1); echo rc=$?; grep -c center_px $out/replay4.jsonl
echo "== final status"; python3 canctl.py status | head -c 300; echo; echo OUT=$out
