#!/usr/bin/env python3
"""
ace-bridge: run several daisy-chained Anycubic ACE 2 Pro units over ONE
USB-RS485 cable, and present each unit as its own virtual serial port.

Why: ACE 2 Pro units chained through their own chain ports share one RS-485
bus. Every unit boots at address 0, so a host that only speaks to address 0
(multiACE today) gets every unit answering every command. The protocol does
support per-unit addresses (DISCOVER_DEVICE / ASSIGN_DEVICE_ID, measured by
tobecwb, multiACE issue #120), but a host has to enumerate the units, keep
each address alive (it lapses after ~3.5 s of silence) and route replies.

This bridge does exactly that and nothing else. To the host each virtual port
looks like a single ACE 2 on its own cable: the host sends frames at address
0x00 and gets replies at 0x80, as it does today. The bridge:

  * finds every ACE 2 cable (USB 1a86:55d3, or --bus PATH), enumerates the
    units on each bus and gives each a distinct address;
  * remembers units by their hardware UID in a small JSON state file, so a
    unit always gets the same virtual port name (ace0, ace1, ...) across
    reboots and power cycles, on any printer, with nothing hard-coded;
  * rewrites the address byte (and the CRC, which covers it) on the way in
    and out, and runs one request at a time on the half-duplex bus;
  * keeps every address alive with a GET_INFO when the host has been quiet;
  * re-assigns a unit that rebooted (dropped back to address 0) and picks up
    units that are switched on later;
  * refuses ASSIGN_DEVICE_ID (would re-address a unit behind its back) and the
    firmware-flash commands IAP_UPGRADE/IAP_FIRMWARE/IAP_FINISH (a unit loses
    its address when it reboots into new firmware mid-flash). Flash a unit on
    its own cable instead.

Wire format (hakimio / tobecwb):
  FF AA | ADDR | SEQ_LO SEQ_HI | CMD | LEN | PAYLOAD | CRC_LO CRC_HI | FE
  ADDR: host->unit 0x0n, unit->host 0x8n.  CRC-16 (poly 0x8408 reflected,
  init 0xFFFF) over ADDR..PAYLOAD.

Licence: MIT. No affiliation with Anycubic.
"""

import argparse
import errno
import fcntl
import json
import logging
import logging.handlers
import os
import select
import signal
import sys
import termios
import time
import tty

VERSION = '0.1.0'

PREAMBLE = b'\xff\xaa'
END = 0xFE
HEADER = 7          # FF AA ADDR SEQ SEQ CMD LEN
TRAILER = 3         # CRC CRC FE
BAUD = 230400

CMD_DISCOVER = 0
CMD_ASSIGN = 1
CMD_IAP_UPGRADE = 2
CMD_IAP_FIRMWARE = 3
CMD_IAP_FINISH = 4
CMD_GET_INFO = 7
CMD_FLASH_LED = 70
BLOCKED_CMDS = {CMD_ASSIGN, CMD_IAP_UPGRADE, CMD_IAP_FIRMWARE, CMD_IAP_FINISH}

ADDR_EXPIRY_S = 3.5      # measured by tobecwb
KEEPALIVE_S = 1.0        # touch every unit at least this often
REPLY_TIMEOUT_S = 1.0    # wait for one host reply
KEEPALIVE_TIMEOUT_S = 0.2  # replies come in <45 ms; keep the bus free
DISCOVER_WAIT_S = 0.20   # collect all broadcast replies (they arrive <45 ms)
RESCAN_S = 5.0           # look for rebooted / newly powered units
LOST_AFTER_FAILS = 2     # consecutive missed replies -> unit considered lost
MAX_UNITS_PER_BUS = 4
INTERNAL_SEQ_BASE = 0xF000  # seq range for the bridge's own frames

V2_USB_IDS = {('1a86', '55d3')}

log = logging.getLogger('ace-bridge')


# ---------------------------------------------------------------- framing --

def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    return crc & 0xFFFF


def build(addr, seq, cmd, payload=b''):
    inner = bytes([addr & 0xFF, seq & 0xFF, (seq >> 8) & 0xFF, cmd & 0xFF,
                   len(payload)]) + payload
    c = crc16(inner)
    return PREAMBLE + inner + bytes([c & 0xFF, (c >> 8) & 0xFF, END])


def readdress(frame, addr):
    """Same frame with a new address byte and a recomputed CRC."""
    inner = bytes([addr & 0xFF]) + frame[3:-3]
    c = crc16(inner)
    return PREAMBLE + inner + bytes([c & 0xFF, (c >> 8) & 0xFF, END])


class FrameParser:
    """Incremental parser. feed() returns every complete CRC-valid frame as
    dicts; garbage and bad-CRC frames are skipped. A broadcast can return
    several frames back to back, so it never stops at the first one."""

    def __init__(self):
        self.buf = bytearray()
        self.crc_errors = 0

    def feed(self, data):
        self.buf += data
        out = []
        while True:
            j = self.buf.find(PREAMBLE)
            if j < 0:
                # keep a trailing 0xFF, it may start the next preamble
                del self.buf[:-1 if self.buf.endswith(b'\xff') else len(self.buf)]
                return out
            if j:
                del self.buf[:j]
            if len(self.buf) < HEADER:
                return out
            plen = self.buf[6]
            total = HEADER + plen + TRAILER
            if len(self.buf) < total:
                return out
            frame = bytes(self.buf[:total])
            if frame[-1] != END or \
                    (frame[-3] | (frame[-2] << 8)) != crc16(frame[2:-3]):
                self.crc_errors += 1
                del self.buf[:2]
                continue
            del self.buf[:total]
            out.append({'raw': frame, 'addr': frame[2],
                        'seq': frame[3] | (frame[4] << 8), 'cmd': frame[5],
                        'payload': frame[HEADER:HEADER + plen]})


# ------------------------------------------------------- minimal protobuf --

def pb_uint32(field, value):
    out = bytes([field << 3])
    while True:
        b, value = value & 0x7F, value >> 7
        out += bytes([b | (0x80 if value else 0)])
        if not value:
            return out


def pb_decode(data):
    def varint(p):
        r = s = 0
        while True:
            b = data[p]
            p += 1
            r |= (b & 0x7F) << s
            if not b & 0x80:
                return r, p
            s += 7
    out, p = {}, 0
    try:
        while p < len(data):
            key, p = varint(p)
            tag, wire = key >> 3, key & 7
            if wire == 0:
                v, p = varint(p)
            elif wire == 2:
                n, p = varint(p)
                v, p = data[p:p + n], p + n
            elif wire == 5:
                v, p = data[p:p + 4], p + 4
            elif wire == 1:
                v, p = data[p:p + 8], p + 8
            else:
                break
            out.setdefault(tag, v)
    except IndexError:
        pass
    return out


def uid_of(payload):
    d = pb_decode(payload)
    uid = (int(d.get(1, 0) or 0), int(d.get(2, 0) or 0), int(d.get(3, 0) or 0))
    return None if uid == (0, 0, 0) else uid


def uid_str(uid):
    return '%d-%d-%d' % uid


# ------------------------------------------------------------------ state --

class State:
    """Units remembered by UID, in first-seen order. The index is the port
    number (ace0, ace1, ...). Edit the file (with the bridge stopped) to
    reorder or forget units."""

    def __init__(self, path):
        self.path = path
        self.units = []      # list of uid strings
        self.labels = {}     # uid -> optional free-text label
        try:
            with open(path) as f:
                data = json.load(f)
            for u in data.get('units', []):
                uid = u['uid'] if isinstance(u, dict) else str(u)
                if uid not in self.units:
                    self.units.append(uid)
                    if isinstance(u, dict) and u.get('label'):
                        self.labels[uid] = u['label']
        except FileNotFoundError:
            pass
        except Exception as e:
            log.warning('state file %s unreadable (%s) - starting empty',
                        path, e)

    def index_of(self, uid):
        if uid not in self.units:
            self.units.append(uid)
            self.save()
            log.info('new unit %s -> ace%d', uid, len(self.units) - 1)
        return self.units.index(uid)

    def save(self):
        data = {'version': 1,
                'note': 'ace-bridge unit order: index = virtual port aceN. '
                        'Stop the bridge before editing.',
                'units': [{'port': 'ace%d' % i, 'uid': u,
                           'label': self.labels.get(u, '')}
                          for i, u in enumerate(self.units)]}
        tmp = self.path + '.tmp'
        with open(tmp, 'w') as f:
            json.dump(data, f, indent=2)
            f.write('\n')
        os.replace(tmp, self.path)


# --------------------------------------------------------- virtual ports --

class VirtualPort:
    """A pty pair. The host opens the slave through the stable symlink; the
    bridge reads/writes the master. The bridge keeps its own handle on the
    slave so the master never sees a hangup while the host has it closed."""

    def __init__(self, index, port_dir):
        self.index = index
        self.name = 'ace%d' % index
        self.master, self.slave = os.openpty()
        tty.setraw(self.slave)
        attrs = termios.tcgetattr(self.slave)
        attrs[3] &= ~termios.ECHO
        termios.tcsetattr(self.slave, termios.TCSANOW, attrs)
        fl = fcntl.fcntl(self.master, fcntl.F_GETFL)
        fcntl.fcntl(self.master, fcntl.F_SETFL, fl | os.O_NONBLOCK)
        self.slave_name = os.ttyname(self.slave)
        os.chmod(self.slave_name, 0o666)
        self.link = os.path.join(port_dir, self.name)
        _symlink(self.slave_name, self.link)
        self.parser = FrameParser()
        log.info('%s -> %s', self.link, self.slave_name)

    def read_frames(self):
        try:
            data = os.read(self.master, 4096)
        except OSError as e:
            if e.errno in (errno.EAGAIN, errno.EIO):
                return []
            raise
        return self.parser.feed(data)

    def write(self, data):
        try:
            os.write(self.master, data)
        except OSError as e:
            if e.errno not in (errno.EAGAIN, errno.EIO):
                raise
            log.debug('%s: host not reading, reply dropped', self.name)

    def add_link(self, path):
        _symlink(self.slave_name, path)


def _symlink(target, link):
    # hidden temp name, so a host glob like ace[0-9]* never matches it
    tmp = os.path.join(os.path.dirname(link), '.' + os.path.basename(link) + '.tmp')
    try:
        os.unlink(tmp)
    except FileNotFoundError:
        pass
    os.symlink(target, tmp)
    os.replace(tmp, link)


# -------------------------------------------------------------------- bus --

class Unit:
    def __init__(self, uid, addr, port):
        self.uid = uid
        self.addr = addr
        self.port = port
        self.present = False
        self.last_tx = 0.0
        self.fails = 0


class Bus:
    """One real USB-RS485 cable with 1..4 chained units behind it."""

    def __init__(self, path, bridge):
        import serial
        self.path = path
        self.bridge = bridge
        # Do not assert DTR/RTS (on some adapters they are wired to a reset).
        # Set them before open; a pty (tests) has no modem lines at all.
        self.ser = serial.Serial()
        self.ser.port, self.ser.baudrate, self.ser.timeout = path, BAUD, 0
        try:
            self.ser.dtr = False
            self.ser.rts = False
        except (OSError, serial.SerialException):
            pass
        self.ser.open()
        time.sleep(0.2)
        self.ser.reset_input_buffer()
        self.parser = FrameParser()
        self.units = {}          # addr -> Unit
        self.by_uid = {}         # uid -> Unit
        self.queue = []          # (unit, frame) from the host
        self.inflight = None     # dict(unit, seq, internal, deadline, port)
        self.internal_seq = INTERNAL_SEQ_BASE
        self.last_scan = 0.0
        log.info('bus %s opened', path)

    # -- low level ----------------------------------------------------------
    def _next_seq(self):
        self.internal_seq += 1
        if self.internal_seq > 0xFFFF:
            self.internal_seq = INTERNAL_SEQ_BASE
        return self.internal_seq

    def _drain(self, wait):
        """Blocking read of every frame that arrives within `wait`."""
        frames, deadline = [], time.monotonic() + wait
        while time.monotonic() < deadline:
            data = self.ser.read(512)
            if data:
                frames += self.parser.feed(data)
            else:
                time.sleep(0.002)
        return frames

    def _transact(self, addr, cmd, payload=b'', wait=0.25):
        """Synchronous request used only for enumeration (bus is idle)."""
        self.ser.reset_input_buffer()
        self.ser.write(build(addr, self._next_seq(), cmd, payload))
        self.ser.flush()
        return [f for f in self._drain(wait) if f['addr'] & 0x80]

    # -- enumeration --------------------------------------------------------
    def scan(self):
        """DISCOVER at address 0 and (re)assign every unit that answers.
        Self-thinning: one clean reply per round is enough."""
        self.last_scan = time.monotonic()
        first = not self.units
        tries = 0
        for _ in range(MAX_UNITS_PER_BUS * 3 + 3):
            self._keepalive_all(sync=True)
            uids = []
            for f in self._transact(0, CMD_DISCOVER, wait=DISCOVER_WAIT_S):
                if f['cmd'] == CMD_DISCOVER:
                    uid = uid_of(f['payload'])
                    if uid and uid not in uids:
                        uids.append(uid)
            if not uids:
                if first and tries < 3:
                    # the first frames after opening the cable can go
                    # unanswered; do not report "no units" on a fluke
                    tries += 1
                    time.sleep(0.2)
                    continue
                return
            first = False
            uid = uid_str(uids[0])
            unit = self.by_uid.get(uid)
            if unit is None:
                addr = self._free_addr()
                if addr is None:
                    log.error('bus %s: more than %d units, ignoring %s',
                              self.path, MAX_UNITS_PER_BUS, uid)
                    return
                port = self.bridge.port_for(uid)
                unit = Unit(uid, addr, port)
                self.units[addr] = unit
                self.by_uid[uid] = unit
            u = [int(x) for x in uid.split('-')]
            payload = (pb_uint32(1, u[0]) + pb_uint32(2, u[1]) +
                       pb_uint32(3, u[2]) + pb_uint32(4, unit.addr))
            # The ACK comes back on the OLD address (0): do not wait for it
            # on the new one.
            self._transact(0, CMD_ASSIGN, payload, wait=0.3)
            ok = any(f['cmd'] == CMD_DISCOVER and uid_of(f['payload'])
                     and uid_str(uid_of(f['payload'])) == uid
                     for f in self._transact(unit.addr, CMD_DISCOVER, wait=0.25))
            if ok:
                unit.present = True
                unit.fails = 0
                unit.last_tx = time.monotonic()
                fw = self._firmware(unit.addr)
                log.info('bus %s: unit %s at address %d -> %s (fw %s)',
                         self.path, uid, unit.addr, unit.port.name, fw)
            else:
                log.warning('bus %s: assign of %s to %d not confirmed, retry',
                            self.path, uid, unit.addr)

    def _firmware(self, addr):
        for f in self._transact(addr, CMD_GET_INFO, wait=0.2):
            v = pb_decode(f['payload']).get(1, b'')
            if isinstance(v, (bytes, bytearray)):
                return v.decode(errors='replace')
        return '?'

    def _free_addr(self):
        for a in range(1, MAX_UNITS_PER_BUS + 1):
            if a not in self.units:
                return a
        return None

    def _keepalive_all(self, sync=False):
        now = time.monotonic()
        for unit in self.units.values():
            if unit.present and now - unit.last_tx > KEEPALIVE_S:
                if sync:
                    self._transact(unit.addr, CMD_GET_INFO, wait=0.06)
                    unit.last_tx = time.monotonic()

    # -- host traffic -------------------------------------------------------
    def host_frame(self, unit, frame):
        if frame['addr'] & 0x80:
            return
        if frame['cmd'] in BLOCKED_CMDS:
            log.warning('%s: host sent blocked command %d - not forwarded '
                        '(flash a unit on its own cable)', unit.port.name,
                        frame['cmd'])
            return
        if len(self.queue) > 64:
            log.warning('%s: queue full, dropping a request', unit.port.name)
            self.queue.pop(0)
        self.queue.append((unit, frame, False))

    def identify(self, unit):
        payload = (pb_uint32(1, 0x0F) + pb_uint32(2, 2) + pb_uint32(3, 0) +
                   pb_uint32(4, 4) + pb_uint32(5, 0) + pb_uint32(6, 0))
        seq = self._next_seq()
        self.queue.append((unit, {'raw': build(0, seq, CMD_FLASH_LED, payload),
                                  'seq': seq}, True))
        log.info('%s: identify (LED blink) queued', unit.port.name)

    def _send(self, unit, frame, internal):
        self.ser.write(readdress(frame['raw'], unit.addr))
        self.ser.flush()
        unit.last_tx = time.monotonic()
        # A unit that stopped answering must not hold the bus for long:
        # every other unit's address lapses after 3.5 s without traffic.
        wait = (KEEPALIVE_TIMEOUT_S if internal or not unit.present
                else REPLY_TIMEOUT_S)
        self.inflight = {'unit': unit, 'seq': frame['seq'],
                         'internal': internal,
                         'deadline': time.monotonic() + wait}

    def _on_bus_frame(self, f):
        if not f['addr'] & 0x80:
            return                       # our own echo, if any
        addr = f['addr'] & 0x7F
        inf = self.inflight
        unit = self.units.get(addr)
        if unit is None:
            if addr == 0:
                log.info('bus %s: a unit answered at address 0 - '
                         'will re-enumerate', self.path)
                self.last_scan = 0.0
            return
        unit.fails = 0
        unit.present = True
        if inf and inf['unit'] is unit and inf['seq'] == f['seq']:
            self.inflight = None
            if inf['internal']:
                return
        elif f['seq'] >= INTERNAL_SEQ_BASE:
            return                       # late reply to a keep-alive
        unit.port.write(readdress(f['raw'], 0x80))

    def poll(self):
        """One non-blocking step: read the bus, time out, send next."""
        data = self.ser.read(4096)
        if data:
            for f in self.parser.feed(data):
                self._on_bus_frame(f)
        now = time.monotonic()
        inf = self.inflight
        if inf and now > inf['deadline']:
            unit = inf['unit']
            unit.fails += 1
            self.inflight = None
            if unit.fails >= LOST_AFTER_FAILS and unit.present:
                unit.present = False
                log.warning('%s (%s): no reply %d times - marked lost, '
                            'rescanning', unit.port.name, unit.uid, unit.fails)
                self.last_scan = 0.0
        if self.inflight:
            return
        if now - self.last_scan > RESCAN_S:
            self.scan()
            return
        while self.queue:
            unit, frame, internal = self.queue.pop(0)
            if not unit.present:
                # lost unit: drop instead of blocking the bus; the host
                # times out on its own, the rescan brings the unit back
                continue
            self._send(unit, frame, internal)
            return
        for unit in self.units.values():
            if unit.present and now - unit.last_tx > KEEPALIVE_S:
                seq = self._next_seq()
                self._send(unit, {'raw': build(0, seq, CMD_GET_INFO),
                                  'seq': seq}, internal=True)
                return

    def fileno(self):
        return self.ser.fileno()


# ----------------------------------------------------------------- bridge --

class Bridge:
    def __init__(self, args):
        self.args = args
        os.makedirs(args.dir, exist_ok=True)
        os.makedirs(os.path.join(args.dir, 'by-uid'), exist_ok=True)
        self.state = State(args.state)
        self.ports = {}          # index -> VirtualPort
        self.buses = []
        self.running = True
        # Ports for every remembered unit exist from the start, so the host
        # always sees the same set in the same order, present or not.
        for uid in self.state.units:
            self.port_for(uid)

    def port_for(self, uid):
        idx = self.state.index_of(uid)
        if idx not in self.ports:
            p = VirtualPort(idx, self.args.dir)
            p.add_link(os.path.join(self.args.dir, 'by-uid', uid))
            self.ports[idx] = p
        return self.ports[idx]

    def open_buses(self):
        paths = self.args.bus or find_buses()
        if not paths:
            log.error('no ACE 2 cable found (USB 1a86:55d3); pass --bus PATH')
        for p in paths:
            try:
                bus = Bus(p, self)
                bus.scan()
                self.buses.append(bus)
            except Exception as e:
                log.error('bus %s: cannot open (%s)', p, e)

    def _open_control(self):
        path = os.path.join(self.args.dir, 'control')
        try:
            os.unlink(path)
        except FileNotFoundError:
            pass
        os.mkfifo(path, 0o660)
        # O_RDWR keeps the FIFO open with no writer, so it never hits EOF.
        self.ctl = os.open(path, os.O_RDWR | os.O_NONBLOCK)
        self.ctl_buf = b''

    def _control(self):
        try:
            self.ctl_buf += os.read(self.ctl, 1024)
        except BlockingIOError:
            return
        while b'\n' in self.ctl_buf:
            line, self.ctl_buf = self.ctl_buf.split(b'\n', 1)
            words = line.decode(errors='replace').split()
            if len(words) == 2 and words[0] == 'identify':
                for bus in self.buses:
                    for unit in bus.units.values():
                        if unit.port.name == words[1] or unit.uid == words[1]:
                            bus.identify(unit)
                            break
                    else:
                        continue
                    break
                else:
                    log.warning('identify: %s is not on any bus', words[1])
            elif words:
                log.warning('control: unknown command %r', line)

    def run(self):
        self._open_control()
        self.open_buses()
        with open(os.path.join(self.args.dir, 'ready'), 'w') as f:
            f.write('%d\n' % os.getpid())
        log.info('ready: %s', ', '.join(
            '%s=%s' % (p.name, p.slave_name) for p in self.ports.values()))
        while self.running:
            rlist = [p.master for p in self.ports.values()] + [self.ctl]
            rlist += [b.fileno() for b in self.buses]
            try:
                select.select(rlist, [], [], 0.02)
            except InterruptedError:
                pass
            self._control()
            for bus in self.buses:
                for unit in list(bus.units.values()):
                    for f in unit.port.read_frames():
                        bus.host_frame(unit, f)
                bus.poll()
            # ports of remembered units not on any bus: swallow host traffic
            on_bus = {u.port.index for b in self.buses for u in b.units.values()}
            for idx, p in self.ports.items():
                if idx not in on_bus:
                    p.read_frames()

    def stop(self, *_):
        self.running = False


def read_usb_ids(tty_name):
    base = os.path.realpath('/sys/class/tty/%s/device' % tty_name)
    for d in (base, os.path.dirname(base)):
        try:
            with open(os.path.join(d, 'idVendor')) as f:
                v = f.read().strip()
            with open(os.path.join(d, 'idProduct')) as f:
                p = f.read().strip()
            return v, p
        except OSError:
            continue
    return None, None


def find_buses():
    out = []
    by_path = '/dev/serial/by-path'
    if not os.path.isdir(by_path):
        return out
    for e in sorted(os.listdir(by_path)):
        full = os.path.join(by_path, e)
        ids = read_usb_ids(os.path.basename(os.path.realpath(full)))
        if ids in V2_USB_IDS:
            out.append(full)
    return out


def main():
    ap = argparse.ArgumentParser(description='ACE 2 Pro daisy-chain bridge')
    ap.add_argument('--bus', action='append',
                    help='ACE cable serial path (repeatable); default: auto')
    ap.add_argument('--dir', default='/tmp/ace-bridge',
                    help='where the virtual port symlinks go')
    ap.add_argument('--state', default='/home/lava/printer_data/config/'
                    'ace-bridge.json', help='unit order file')
    ap.add_argument('--log', default='/home/lava/printer_data/logs/'
                    'ace-bridge.log')
    ap.add_argument('--debug', action='store_true')
    ap.add_argument('--identify', metavar='aceN',
                    help='ask the RUNNING bridge to blink that unit, then exit')
    args = ap.parse_args()

    if args.identify:
        ctl = os.path.join(args.dir, 'control')
        if not os.path.exists(os.path.join(args.dir, 'ready')):
            sys.exit('bridge is not running (no %s/ready)' % args.dir)
        fd = os.open(ctl, os.O_WRONLY | os.O_NONBLOCK)
        os.write(fd, ('identify %s\n' % args.identify).encode())
        os.close(fd)
        print('asked the bridge to blink %s' % args.identify)
        return

    handlers = [logging.StreamHandler(sys.stderr)]
    if args.log:
        try:
            handlers.append(logging.handlers.RotatingFileHandler(
                args.log, maxBytes=1_000_000, backupCount=2))
        except OSError:
            pass
    logging.basicConfig(level=logging.DEBUG if args.debug else logging.INFO,
                        format='%(asctime)s %(levelname)s %(message)s',
                        handlers=handlers)
    log.info('ace-bridge %s starting', VERSION)

    bridge = Bridge(args)
    signal.signal(signal.SIGTERM, bridge.stop)
    signal.signal(signal.SIGINT, bridge.stop)
    try:
        bridge.run()
    finally:
        try:
            os.unlink(os.path.join(args.dir, 'ready'))
        except OSError:
            pass
        log.info('ace-bridge stopped')


if __name__ == '__main__':
    main()
