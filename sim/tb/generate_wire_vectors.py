#!/usr/bin/python3
"""Independent classic extended-CAN serializer for receive/error-injection tests."""
from pathlib import Path
import json

def bits(value, width):
    return [(value >> i) & 1 for i in range(width-1, -1, -1)]

def frame(identifier, payload, corrupt_crc=False):
    # SOF, base identifier, SRR, IDE, extended identifier, RTR, r1, r0, DLC.
    stream = [0]+bits(identifier >> 18,11)+[1,1]+bits(identifier & 0x3ffff,18)+[0,0,0]+bits(len(payload),4)
    for byte in payload: stream += bits(byte,8)
    crc=0
    for bit in stream:
        feedback=((crc >> 14)&1)^bit
        crc=(crc << 1)&0x7fff
        if feedback: crc ^= 0x4599
    if corrupt_crc: crc ^= 1
    raw=stream+bits(crc,15)
    stuffed=[];last=None;run=0
    for bit in raw:
        stuffed.append(bit)
        run=run+1 if bit==last else 1
        last=bit
        if run==5:
            last=1-bit;stuffed.append(last);run=1
    # CRC delimiter, ACK slot (recessive sender), ACK delimiter, EOF, intermission.
    return stuffed+[1]*13

if __name__=='__main__':
    root=Path(__file__).resolve().parents[1]/'result';root.mkdir(exist_ok=True)
    valid=frame(0x028002fd,bytes.fromhex('8000123456780190'))
    invalid=frame(0x028003fd,bytes.fromhex('8000123456780190'),True)
    for name,stream in [('wire_valid',valid),('wire_bad_crc',invalid)]:
        (root/(name+'.mem')).write_text('\n'.join(str(b) for b in stream)+'\n')
    (root/'wire_lengths.vh').write_text(f'`define VALID_BITS {len(valid)}\n`define BAD_BITS {len(invalid)}\n')
    (root/'wire_vectors.json').write_text(json.dumps({'valid_bits':len(valid),'bad_crc_bits':len(invalid),'crc_polynomial':'0x4599'},indent=2)+'\n')
