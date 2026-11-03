#!/usr/bin/python3
"""Independent integer model of 40-bit-fraction Horner and Q8.24 -> float."""
from pathlib import Path
import struct
root=Path(__file__).resolve().parent
Q=1<<24
rows=[]
for axis in range(7):rows.append([round((-0.25+axis/32)*Q),0,0,0,round(8.75*Q),-21*Q,round(17.5*Q),-5*Q])
(root/'wide_coefficients.mem').write_text(''.join(f'{x&0xffffffff:08x}\n' for row in rows for x in row))
expected=[]
for tick in range(21):
    phase=tick*Q//20
    for row in rows:
        y=row[7]<<16
        for c in reversed(row[:7]):
            # round(int / 2^24) ties to even, using integers only
            z=y*phase;whole,rem=divmod(z,Q)
            y=whole+(rem>Q//2 or (rem==Q//2 and whole%2))+(c<<16)
        whole,rem=divmod(y,1<<16)
        q=whole+(rem>(1<<15) or (rem==(1<<15) and whole%2))
        expected.append(struct.unpack('>I',struct.pack('>f',q/Q))[0])
(root/'wide_expected.mem').write_text(''.join(f'{x:08x}\n' for x in expected))
print('Generated 147 degree-seven packet reference values')
