#!/usr/bin/python3
"""Independent floating-reference vectors for the hybrid Type1/CSP testbench."""
import argparse,math,struct
from pathlib import Path
Q=1<<24
def generate():
    q=[[.5+i*.001,0,0,0,.35,-.84,.7,-.2] for i in range(6)]+[[1.6]+[0]*7]
    v=[[.5,.01,-.01]+[0]*5 for _ in range(7)]
    t=[[.4+i*.001,.05]+[0]*6 for i in range(7)]
    stage=[round(c*Q) for field in (q,v,t) for row in field for c in row]
    for field in range(3):stage += [0]*7+[Q]*6+[2*Q if field==0 else Q]
    stage += [0x28f5cccc]*6+[0,4,Q//4,0]
    packets=[]
    for tick in range(5):
        u=tick/4
        sample=lambda row:sum(c*u**i for i,c in enumerate(row))
        enc=lambda x:math.floor(x*65535+.5)
        for axis in range(7):
            if axis<6:
                ident=0x1000000+(enc(sample(t[axis]))<<8)+axis+1
                data=struct.pack('>HHHH',enc(sample(q[axis])),enc(sample(v[axis])),0x28f5,0xcccc)
            else:ident=0x1200fd07;data=bytes.fromhex('16700000')+struct.pack('<f',1.6)
            packets.append(f'{ident:08x}'+data.hex())
    return ''.join(f'{w&0xffffffff:08x}\n' for w in stage),'\n'.join(packets)+'\n'

if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--check',action='store_true');a=ap.parse_args()
    for name,contents in zip(('mit_stage.mem','mit_packets.mem'),generate()):
        p=Path(__file__).with_name(name)
        if a.check:
            if p.read_text()!=contents:raise SystemExit(f'vector mismatch: {name}')
        else:p.write_text(contents)
    print('PASS: 220 staging words and 35 independent hybrid CAN packets')
