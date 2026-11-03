#!/usr/bin/python3
"""Explicit continuous-cubic mode for unchanged v5 HLS hardware.

After the first segment the next encoded sample is 5 ms into its polynomial;
this avoids v5 sending the shared endpoint twice. Bounds include that interval.
"""
from pl_transport import PlTransport,segment_words
from continuous_trajectory import cubic_words,trim_first_tick
from joint_trajectory import Q24

class ContinuousTransport(PlTransport):
    def __init__(self,dev,planner):
        super().__init__(dev,planner);self.tail_velocity=None;self.last_encoded_plan=None

    def submit(self,plan):
        if plan['kind']=='MOVEJ_REST_TO_REST':
            words,ends=segment_words(plan,self.planner);v0=v1=[0.]*7
        else:words,ends,v0,v1=cubic_words(plan,self.planner)
        d=self.dev;s=self.status()
        if d.read(0x404)&3!=3 or d.read(0x418):raise RuntimeError('CAN stopped/faulted')
        if s['fault'] or s['flags']&34:raise RuntimeError('queue full or pause-at-end latched')
        if self.tail is not None and any(abs(words[i*4]-self.tail[i])>2 for i in range(7)):
            raise ValueError('discontinuous position at queued boundary')
        if self.tail_velocity is not None and any(abs(a-b)>2e-4 for a,b in zip(v0,self.tail_velocity)):
            raise ValueError('discontinuous velocity at queued boundary')
        if self.tail is None and max(map(abs,v0))>2e-4:raise ValueError('first segment must start at rest')
        encoded=plan
        if self.tail is not None:
            encoded=trim_first_tick(self.planner,plan)
            encoded_words,encoded_ends,sv,ev=cubic_words(encoded,self.planner)
            # A bounded 5 ms advance, not an arbitrary discontinuity exemption.
            for i in range(7):
                expected=self.tail[i]/Q24+self.tail_velocity[i]*.005
                if abs(encoded_words[i*4]/Q24-expected)>.5*self.planner.acceleration[i]*.005**2+4/Q24:
                    raise ValueError('invalid intersample position advance')
                if abs(sv[i]-self.tail_velocity[i])>self.planner.acceleration[i]*.005+2e-4:
                    raise ValueError('invalid intersample velocity advance')
                if abs(encoded_ends[i]-ends[i])>2:raise ValueError('trim changed endpoint')
            words,ends,v1=encoded_words,encoded_ends,ev
        before=s['rejected']
        for i,word in enumerate(words):d.write(0x900+4*i,word)
        d.write(0x820,1)
        if d.read(0x820)!=before or d.read(0x404)&3!=3 or d.read(0x418):raise RuntimeError('PL rejected commit')
        self.tail=ends;self.tail_velocity=v1;self.last_encoded_plan=encoded
