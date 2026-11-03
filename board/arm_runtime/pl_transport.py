#!/usr/bin/python3
"""v5 PL transport: ARM writes cubic segments, never samples/CAN targets.

Commands are explicit; importing this module never opens UIO. The existing
CanDevice is supplied by the sole owner of the board session. PauseAtEnd is
not an immediate controlled stop. A link/lease fault still invokes motor timeout.
"""
import math
from joint_trajectory import Q24


def extrema(c):
    a,b,d=3*c[3],2*c[2],c[1]
    points=[0.,1.]
    if a:
        disc=b*b-4*a*d
        if disc>=0:
            points.extend(x for x in ((-b+math.sqrt(disc))/(2*a),(-b-math.sqrt(disc))/(2*a)) if 0<x<1)
    elif b:
        x=-d/b
        if 0<x<1:points.append(x)
    values=[((c[3]*x+c[2])*x+c[1])*x+c[0] for x in points]
    return min(values),max(values)


def segment_words(plan, planner):
    if plan['profile_id']!=planner.profile_id or plan['kind']!='MOVEJ_REST_TO_REST':
        raise ValueError('Only matching rest-to-rest MoveJ profile commissioned')
    if plan['tick_s']!=.005 or type(plan['ticks']) is not int or not 1<=plan['ticks']<=120000:
        raise ValueError('v5 fixed 5 ms period and 1..120000 ticks required')
    starts=planner.to_motor(plan['start_rad']);goals=planner.to_motor(plan['goal_rad'])
    lows=[min(a*d+o,b*d+o) for a,b,d,o in zip(planner.lower,planner.upper,planner.direction,planner.offset)]
    highs=[max(a*d+o,b*d+o) for a,b,d,o in zip(planner.lower,planner.upper,planner.direction,planner.offset)]
    words=[];bounds=[];ends=[]
    duration=plan['ticks']*.005
    for i,(a,b,lo,hi) in enumerate(zip(starts,goals,lows,highs)):
        # Quantize endpoints toward the legal interval, by at most one Q24 LSB.
        l,h=math.ceil(lo*Q24),math.floor(hi*Q24)
        start=max(l,min(h,round(a*Q24)));end=max(l,min(h,round(b*Q24)))
        if max(abs(a-start/Q24),abs(b-end/Q24))>1/Q24:raise ValueError('endpoint quantization')
        d=end-start;c=[start,0,3*d,-2*d]
        if any(not -(1<<31)<=v<1<<31 for v in c):raise ValueError('coefficient overflow')
        minimum,maximum=extrema(c)
        if minimum<l or maximum>h:raise ValueError('quantized polynomial exceeds limits')
        if 1.5*abs(d)/Q24/duration>planner.velocity[i]+1e-6 or 6*abs(d)/Q24/duration**2>planner.acceleration[i]+1e-5:
            raise ValueError('quantized derivative bound exceeded')
        words.extend(c);bounds.append((l,h));ends.append(end)
    words += [x[0] for x in bounds]+[x[1] for x in bounds]
    n=plan['ticks'];step,rem=divmod(Q24,n)
    words += [n,step,rem]
    return words,ends


class PlTransport:
    def __init__(self,dev,planner):
        if dev.read(0x470) not in (0x50000,0x50001,0x60000,0x60001,0x60002,0x60003):raise RuntimeError('v5 trajectory firmware required')
        self.dev,self.planner=dev,planner;self.tail=None

    def status(self):
        d=self.dev
        return dict(flags=d.read(0x800),fault=d.read(0x804),batches=d.read(0x808),
                    completed=d.read(0x80c),tick=d.read(0x810),ticks=d.read(0x814),
                    holding=bool(d.read(0x81c)),rejected=d.read(0x820),mask=d.read(0x824))

    def submit(self,plan):
        words,ends=segment_words(plan,self.planner)
        d=self.dev;s=self.status()
        if d.read(0x404)&3!=3 or d.read(0x418):raise RuntimeError('CAN stopped/faulted')
        if s['fault'] or s['flags']&34:raise RuntimeError('queue full or pause-at-end latched')
        if self.tail is not None and any(abs(words[i*4]-self.tail[i])>1 for i in range(7)):
            raise ValueError('discontinuous queued segment start')
        before=s['rejected']
        for i,word in enumerate(words):d.write(0x900+4*i,word)
        d.write(0x820,1)
        # A holding engine may consume the new queue entry before the status read.
        # Rejection count and controller/fault state acknowledge the transaction.
        if d.read(0x820)!=before or d.read(0x404)&3!=3 or d.read(0x418):
            raise RuntimeError('PL rejected atomic commit or controller stopped')
        self.tail=ends

    def start(self):
        s=self.status()
        if s['flags']&1 or not s['flags']&2:raise RuntimeError('start requires idle committed segment')
        self.dev.write(0x820,2)
        if not self.dev.read(0x800)&1:raise RuntimeError('PL refused start')

    def pause_at_end(self):
        if not self.dev.read(0x800)&1:raise RuntimeError('not running')
        self.dev.write(0x820,4)
