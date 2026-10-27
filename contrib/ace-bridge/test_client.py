#!/usr/bin/env python3
"""Plays multiACE: opens each virtual port like a lone ACE 2 (frames at
address 0, expects replies at 0x80) and checks the bridge routes correctly.
usage: test_client.py PORT_DIR [expect_units]"""

import os
import sys
import time

import serial

sys.path.insert(0, os.path.dirname(__file__))
from ace_bridge import (FrameParser, build, pb_decode, uid_of, uid_str,
                        CMD_DISCOVER, CMD_GET_INFO, CMD_ASSIGN)

d = sys.argv[1]
want = int(sys.argv[2]) if len(sys.argv) > 2 else 2
ports = sorted(p for p in os.listdir(d) if p.startswith('ace') and
               not p.endswith('.tmp'))
assert len(ports) >= want, ports
conns = {p: (serial.Serial(os.path.join(d, p), 230400, timeout=0),
             FrameParser()) for p in ports[:want]}
seq = [100]


def ask(p, cmd, wait=1.5):
    ser, parser = conns[p]
    seq[0] += 1
    ser.write(build(0x00, seq[0], cmd))
    end = time.monotonic() + wait
    while time.monotonic() < end:
        for f in parser.feed(ser.read(512)):
            assert f['addr'] == 0x80, 'reply address %#x' % f['addr']
            if f['seq'] == seq[0]:
                return f
        time.sleep(0.005)
    return None


def info(p):
    f = ask(p, CMD_GET_INFO)
    return pb_decode(f['payload']).get(1, b'?').decode() if f else None


def who(p):
    f = ask(p, CMD_DISCOVER)
    return uid_str(uid_of(f['payload'])) if f else None


ok = True
mapping = {p: (who(p), info(p)) for p in conns}
print('mapping', mapping)
ok &= all(v[0] and v[1] for v in mapping.values())
ok &= len({v[0] for v in mapping.values()}) == len(mapping)
# interleaved traffic on both ports
for i in range(20):
    for p in conns:
        if who(p) != mapping[p][0]:
            print('MISROUTE', p, i)
            ok = False
print('interleaved 40 requests ok' if ok else 'interleaved FAILED')
# blocked command must not reach the bus (no reply)
f = ask(ports[0], CMD_ASSIGN, wait=0.6)
print('ASSIGN blocked' if f is None else 'ASSIGN WAS FORWARDED')
ok &= f is None
# idle longer than the 3.5 s expiry: keep-alive must hold the addresses
time.sleep(6)
after = {p: who(p) for p in conns}
same = all(after[p] == mapping[p][0] for p in conns)
print('after 6 s idle', 'addresses held' if same else after)
ok &= same
print('RESULT', 'PASS' if ok else 'FAIL')
with open(os.path.join(d, 'client.mapping'), 'w') as fh:
    fh.write(repr(mapping))
sys.exit(0 if ok else 1)
