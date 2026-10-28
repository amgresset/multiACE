#!/usr/bin/env python3
"""Simulated chain of ACE 2 Pro units on one pty, for testing ace_bridge.py.

Behaviour copied from tobecwb's measurements: units boot at address 0,
answer DISCOVER with their UID, ACK an ASSIGN on the OLD address, drop back to
address 0 after 3.5 s without a frame addressed to them, and serialise their
replies to a broadcast. Writes the slave path to argv[1] and serves forever.
Send SIGUSR1 to 'reboot' unit 2 (addr -> 0), SIGUSR2 to make unit 1
ignore frames for 2 s while keeping its address."""

import os
import random
import select
import signal
import sys
import time
import tty

sys.path.insert(0, os.path.dirname(__file__))
from ace_bridge import (FrameParser, build, pb_uint32, pb_decode, uid_of,
                        CMD_DISCOVER, CMD_ASSIGN, CMD_GET_INFO, CMD_FLASH_LED)


class SimUnit:
    def __init__(self, uid, fw):
        self.uid, self.fw, self.addr, self.last = uid, fw, 0, time.monotonic()
        self.cmds = []
        self.mute_until = 0.0     # SIGUSR2: ignore frames, keep address

    def tick(self):
        if self.addr and time.monotonic() - self.last > 3.5:
            self.addr = 0

    def handle(self, f):
        """Return reply frame or None."""
        self.tick()
        if f['addr'] != self.addr:
            return None
        if time.monotonic() < self.mute_until:
            return None           # busy: no reply, but the address is kept
        self.last = time.monotonic()
        self.cmds.append(f['cmd'])
        reply_addr = 0x80 | self.addr
        if f['cmd'] == CMD_DISCOVER:
            p = b''.join(pb_uint32(i + 1, v) for i, v in enumerate(self.uid))
        elif f['cmd'] == CMD_ASSIGN:
            d = pb_decode(f['payload'])
            if uid_of(f['payload']) != self.uid:
                return None
            reply = build(reply_addr, f['seq'], f['cmd'])   # on OLD address
            self.addr = int(d.get(4, 0))
            return reply
        elif f['cmd'] == CMD_GET_INFO:
            p = bytes([0x0A, len(self.fw)]) + self.fw.encode()
        else:
            p = b''
        return build(reply_addr, f['seq'], f['cmd'], p)


def main():
    units = [SimUnit((111111111, 222222, 333333333), 'V1.1.60O'),
             SimUnit((444444444, 555555, 666666666), 'V1.1.31')]
    master, slave = os.openpty()
    tty.setraw(slave)
    with open(sys.argv[1], 'w') as fh:
        fh.write(os.ttyname(slave))
    signal.signal(signal.SIGUSR1, lambda *_: setattr(units[1], 'addr', 0))
    signal.signal(signal.SIGUSR2, lambda *_: setattr(units[0], 'mute_until',
                                                     time.monotonic() + 2.0))
    parser = FrameParser()
    while True:
        r, _, _ = select.select([master], [], [], 0.05)
        for u in units:
            u.tick()
        if not r:
            continue
        for f in parser.feed(os.read(master, 4096)):
            if f['addr'] & 0x80:
                continue
            replies = [x for x in (u.handle(f) for u in units) if x]
            random.shuffle(replies)
            for x in replies:            # serialised, like the real bus
                time.sleep(random.uniform(0.005, 0.04))
                os.write(master, x)
        with open(sys.argv[1] + '.log', 'w') as fh:
            fh.write(repr([(u.addr, u.cmds[-12:]) for u in units]))


if __name__ == '__main__':
    main()
