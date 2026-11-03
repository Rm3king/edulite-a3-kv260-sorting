"""Exhaust actual HLS phases using the already verified native integer core.

Only native_fields (pure arithmetic) is called. No MMIO or transport operation.
The slow independent Python implementation remains the regression oracle.
"""
import ctypes
import math


class FieldValidator:
    def __init__(self, library):
        self.lib=ctypes.CDLL(str(library))
        self.lib.native_fields.argtypes=[ctypes.POINTER(ctypes.c_int32),ctypes.c_uint32,ctypes.POINTER(ctypes.c_int32)]
        self.lib.native_fields.restype=ctypes.c_int
        self.checked_phases=0
        self.batch=getattr(self.lib,'native_validate_record',None)
        if self.batch is not None:
            self.batch.argtypes=[ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int32),ctypes.c_double,ctypes.c_double,ctypes.c_int,ctypes.POINTER(ctypes.c_uint32)]
            self.batch.restype=ctypes.c_int
        self.implementation='native_segment_batch' if self.batch is not None else 'scalar_ffi_compatibility'

    def __call__(self, records, job, p, model):
        from gravity_trajectory import encoded_interval
        from joint_trajectory import Q24
        guards=[encoded_interval(job,p,model,i) for i in range(6)]
        bounds=[(guards[i][f]&65535,guards[i][f]>>16) for f in range(3) for i in range(6)]
        buf=(ctypes.c_int32*21)()
        packed_bounds=(ctypes.c_int32*36)(*(v for pair in bounds for v in pair))
        for record in records:
            words=record['words']
            if len(words)!=220 or not 0<=words[216]<=512:raise ValueError('invalid normalized record')
            raw=(ctypes.c_int32*220)(*words)
            if self.batch is not None:
                checked=ctypes.c_uint32()
                rc=self.batch(raw,packed_bounds,job['motor_lower_rad'][6],job['motor_upper_rad'][6],
                    int(max(map(abs,record['end_velocity']))<1e-7),ctypes.byref(checked))
                self.checked_phases+=checked.value
                errors={-1:'native HLS atomic interval failure',-2:'encoded field outside independent guard',-3:'gripper outside reviewed interval',-4:'non-rest normalized final velocity',-5:'invalid normalized record'}
                if rc:raise ValueError(errors.get(rc,'native validation failure '+str(rc)))
                continue
            for tick in range(words[217]+1):
                if self.lib.native_fields(raw,tick,buf):raise ValueError('native HLS atomic interval failure')
                for f in range(3):
                    for i in range(6):
                        code=(int(buf[7*f+i])*65535+(1<<23))>>24
                        lo,hi=bounds[6*f+i]
                        if not lo<=code<=hi:raise ValueError('encoded field outside independent guard')
                if not job['motor_lower_rad'][6]<=buf[6]/Q24<=job['motor_upper_rad'][6]:
                    raise ValueError('gripper outside reviewed interval')
                self.checked_phases+=1
            if max(map(abs,record['end_velocity']))<1e-7:
                if any(((int(buf[7+i])*65535+(1<<23))>>24)!=32768 for i in range(6)):
                    raise ValueError('non-rest normalized final velocity')
        return True
