#!/usr/bin/python3
"""Atomic degree-seven segment transport; version guard precedes new MMIO."""
from pl_transport import PlTransport
from septic_trajectory import septic_words,trim_first_tick
from joint_trajectory import Q24


class SepticTransport(PlTransport):
    def __init__(self,dev,planner,prepared=None):
        # Old FPGA unmapped reads can SIGBUS. Never probe a new capability
        # register unless its firmware version explicitly supports that address.
        if dev.read(0x470) not in (0x50001,0x60000,0x60001,0x60002,0x60003):raise RuntimeError('degree-seven firmware 0x50001 required')
        if dev.read(0x828)!=0x00070008:raise RuntimeError('degree-seven capability mismatch')
        super().__init__(dev,planner)
        self.tail_velocity=None;self.tail_derivatives=None;self.last_encoded_plan=None
        self.prepared=prepared;self.prepared_index=0;self.records=[]

    def submit(self,plan):
        if self.prepared is None:
            words,ends,begin,end=septic_words(plan,self.planner)
            starts=tuple(words[i*8] for i in range(7))
        else:
            if self.prepared_index>=len(self.prepared) or self.prepared[self.prepared_index]['plan'] is not plan:
                raise ValueError('C3 segment not in prevalidated execution order')
            record=self.prepared[self.prepared_index]
            words,ends,begin,end=record['words'],record['ends'],record['begin'],record['end']
            starts=record['starts']
        d=self.dev;s=self.status()
        if d.read(0x404)&3!=3 or d.read(0x418):raise RuntimeError('CAN stopped/faulted')
        if s['fault'] or s['flags']&34:raise RuntimeError('queue full or pause-at-end latched')
        if self.tail is not None:
            if any(abs(starts[i]-self.tail[i])>2 for i in range(7)):raise ValueError('C3 position discontinuity')
            for i in range(7):
                for order,tolerance in enumerate((2e-4,.001,.01)):
                    if abs(begin[i][order]-self.tail_derivatives[i][order])>tolerance:
                        raise ValueError('C3 quantized derivative discontinuity')
        elif any(abs(row[k])>(2e-4,.001,.01)[k] for row in begin for k in range(3)):
            raise ValueError('C3 first segment must start at rest through jerk')
        encoded=plan if self.prepared is None else record['encoded_plan']
        if self.tail is not None and self.prepared is None:
            # Encode/validate once immediately below, rather than both inside
            # the constructor and again here. No unchecked record is emitted.
            encoded=trim_first_tick(self.planner,plan,_validate=False)
            encoded_words,encoded_ends,sv,ev=septic_words(encoded,self.planner)
            for i in range(7):
                expected=self.tail[i]/Q24+self.tail_velocity[i]*.005
                if abs(encoded_words[i*8]/Q24-expected)>.5*self.planner.acceleration[i]*.005**2+4/Q24:
                    raise ValueError('C3 invalid 5 ms position advance')
                if abs(sv[i][0]-self.tail_velocity[i])>self.planner.acceleration[i]*.005+2e-4:
                    raise ValueError('C3 invalid 5 ms velocity advance')
                if abs(encoded_ends[i]-ends[i])>2:raise ValueError('C3 trim changed endpoint')
            words,ends,end=encoded_words,encoded_ends,ev
        # Expensive roots, polynomial restriction and quantization are permitted
        # only in the offline path. Live execution receives these immutable
        # records before opening UIO; the object identity check prevents replay
        # or substituting another segment while motors are enabled.
        if self.prepared is None:
            record=dict(plan=plan,encoded_plan=encoded,words=tuple(words),ends=tuple(ends),starts=starts,
                begin=tuple(tuple(row) for row in begin),end=tuple(tuple(row) for row in end))
            self.records.append(record)
        before=s['rejected']
        for i,word in enumerate(words):d.write(0xa00+4*i,word)
        d.write(0x820,8)
        if d.read(0x820)!=before or d.read(0x404)&3!=3 or d.read(0x418):raise RuntimeError('PL rejected degree-seven commit')
        self.tail=ends;self.tail_derivatives=end;self.tail_velocity=[row[0] for row in end]
        self.last_encoded_plan=encoded
        if self.prepared is not None:self.prepared_index+=1
