#!/usr/bin/python3
"""v6 FPGA q/v/tau transport. Only immutable, prevalidated records may run."""
from joint_trajectory import Q24


class GravityTransport:
    phase_register_base=0xb40
    def __init__(self,dev,planner,prepared):
        version=dev.read(0x470)
        if version not in (0x60000,0x60001,0x60002,0x60003):raise RuntimeError('gravity firmware v6 required before new MMIO')
        if version not in (0x60002,0x60003) and any(r['words'][216] for r in prepared):raise RuntimeError('PL damping requires v6.2')
        if dev.read(0xb68)!=0x70003:raise RuntimeError('gravity capability mismatch')
        self.supports_damping=version in (0x60002,0x60003)
        self.dev,self.planner,self.prepared=dev,planner,prepared
        self.prepared_index=0;self.tail=None;self.tail_velocity=None;self.last_encoded_plan=None

    def status(self):
        d=self.dev
        return dict(flags=d.read(0xb40),fault=d.read(0xb44),batches=d.read(0xb48),completed=d.read(0xb4c),
            tick=d.read(0xb50),ticks=d.read(0xb54),holding=bool(d.read(0xb5c)),rejected=d.read(0xb60),mask=d.read(0xb64))

    def submit(self,plan):
        if self.prepared_index>=len(self.prepared):raise ValueError('all gravity segments already consumed')
        r=self.prepared[self.prepared_index]
        if r['words'][216] and not self.supports_damping:raise RuntimeError('PL damping requires v6.2')
        if r['original_plan'] is not plan:raise ValueError('gravity segment not in reviewed order')
        s=self.status();d=self.dev
        if s['fault'] or s['flags']&34 or d.read(0x404)&3!=3 or d.read(0x418):raise RuntimeError('MIT stream/controller unavailable')
        before=s['rejected']
        for i,word in enumerate(r['words']):d.write(0xc00+i*4,word)
        d.write(0xb60,1)
        if d.read(0xb60)!=before or d.read(0x418) or d.read(0x404)&3!=3:raise RuntimeError('MIT segment rejected')
        self.tail=r['ends'];self.tail_velocity=r['end_velocity'];self.last_encoded_plan=r['plan'];self.prepared_index+=1

    def start(self):
        s=self.status()
        if s['flags']&1 or not s['flags']&2:raise RuntimeError('MIT start requires idle queued segment')
        self.dev.write(0xb60,2)
        if not self.dev.read(0xb40)&1:raise RuntimeError('MIT start rejected')
