#!/usr/bin/python3
"""Compare generated command bytes to the existing vendor SDK without hardware."""
import importlib.util,json,os,random,sys,types,unittest
from pathlib import Path
from a3_protocol import motion,quantize
workspace=Path(__file__).resolve().parents[4]
sdk_candidates=([Path(os.environ['A3_SDK_DIR'])] if os.environ.get('A3_SDK_DIR') else
                [workspace/'EDULITE_A3/el_a3_sdk/el_a3_sdk',
                 workspace/'EDULITE_A3_0211abadec527575/el_a3_sdk/el_a3_sdk'])
SDK=next((p for p in sdk_candidates if (p/'protocol.py').is_file() and (p/'can_driver.py').is_file()),None)
if SDK is None:raise RuntimeError('Vendor SDK missing: set A3_SDK_DIR to its el_a3_sdk package directory')
pkg=types.ModuleType('el_a3_sdk');pkg.__path__=[str(SDK)];sys.modules['el_a3_sdk']=pkg
for name in ('protocol','can_driver'):
    spec=importlib.util.spec_from_file_location('el_a3_sdk.'+name,SDK/(name+'.py'))
    module=importlib.util.module_from_spec(spec);sys.modules[spec.name]=module;spec.loader.exec_module(module)
from el_a3_sdk.can_driver import RobstrideCanDriver
from el_a3_sdk.protocol import MotorType
class TestProtocol(unittest.TestCase):
    def test_sdk_comparison(self):
        rng=random.Random(260);count=0
        for model,vmax,tmax in [('RS00',33,14),('EL05',50,6),('RS05',50,5.5)]:
            driver=RobstrideCanDriver(motor_type_map={i:getattr(MotorType,model) for i in range(1,8)})
            captured=[];driver._send_frame=lambda ident,payload:captured.append((ident,payload)) or True
            for motor in range(1,8):
                for i in range(30):
                    values=[rng.uniform(-12.57,12.57),rng.uniform(-vmax,vmax),rng.uniform(0,500),rng.uniform(0,5),rng.uniform(-tmax,tmax)]
                    driver.send_motion_control(motor,*values)
                    self.assertEqual(captured[-1][0] & 0xe0000000,0x80000000) # SocketCAN EFF metadata
                    self.assertEqual(motion(motor,*values,model=model),(captured[-1][0]&0x1fffffff,captured[-1][1]));count+=1
        print('SDK_COMMAND_MATCHES',count)
    def test_reject_invalid(self):
        for value in (float('nan'),float('inf'),-13,13):
            with self.assertRaises(ValueError):motion(1,value,0,1,1)
        for motor in (0,8,1.2):
            with self.assertRaises(ValueError):motion(motor,0,0,1,1)
    def test_quantization_boundaries(self):
        self.assertEqual(quantize(-12.57,-12.57,12.57),0)
        self.assertEqual(quantize(12.57,-12.57,12.57),65535)
if __name__=='__main__':unittest.main()
