# ace-bridge: daisy-chained ACE 2 Pro on one cable

Run several **Anycubic ACE 2 Pro** units chained through their own chain ports
on **one** USB-RS485 cable (the K3 cable), and present each unit to multiACE as
if it had its own cable.

Status: experimental, version 0.1.0. Tested on one Snapmaker U1 with two units.
See [What is tested](#what-is-tested).

## Why

ACE 2 Pro units on a chain share one RS-485 bus behind a single serial port.
Every unit boots at address 0, so a host that only talks to address 0 gets every
unit answering, and acting on, every command. The protocol does support
per-unit addresses. tobecwb measured it on two units in
[multiACE #120](https://github.com/decay71/multiACE/issues/120):

- `DISCOVER` at address 0 makes every unaddressed unit reply with its UID.
- `ASSIGN` gives one UID an address. Its acknowledgement comes back on the
  **old** address.
- An address lapses after **~3.5 s** without a frame addressed to it, and the
  unit drops back to address 0.
- Replies to a broadcast arrive back to back, one per unit.

multiACE today keeps one serial connection per ACE and sends every frame to
address 0. Teaching its core to share one port between several units is a
large change. ace-bridge does it outside multiACE instead.

## How it works

```
           multiACE (unchanged apart from v2_serial_paths)
          /                          \
   /tmp/ace-bridge/ace0        /tmp/ace-bridge/ace1      (virtual ports)
   frames at 0x00              frames at 0x00
          \                          /
     ┌──────────── ace-bridge ─────────────┐
     │ 0x00 -> unit address, fix CRC        │
     │ 0x8n -> 0x80, fix CRC, route reply   │
     │ one request at a time (half-duplex)  │
     │ keep-alive every 1 s per unit        │
     │ re-assign units that reboot          │
     └──────────────────┬──────────────────┘
                        │ one cable, RS-485, 230400 baud
             ACE 2 (top) ──chain── ACE 2 (bottom)
```

- **Finds every ACE 2 cable** by USB ID `1a86:55d3`, or the paths given with
  `--bus`, and enumerates the units on each one, up to 4 per cable.
- **Remembers units by hardware UID** in `config/ace-bridge.json`, first-seen
  order. Index N is port `aceN`, so each unit always gets the same port. Nothing
  is hard-coded, and you can edit the file (with the bridge stopped) to reorder
  units or add labels.
- **Ports exist for every remembered unit,** present or not, so multiACE's ACE
  numbering never shifts.
- **Does not answer on behalf of units,** except swallowing its own keep-alive
  replies.
- **Refuses some commands.** `ASSIGN_DEVICE_ID` (1) would re-address a unit
  behind its back. The firmware-flash commands `IAP_UPGRADE`/`IAP_FIRMWARE`/
  `IAP_FINISH` (2-4) are refused because a unit loses its address when it
  reboots into new firmware mid-flash. **Flash a unit on its own cable.**
- **Identify:** `python3 ace_bridge.py --identify ace1` asks the running bridge
  to blink that unit's LEDs.

## Install on a Snapmaker U1 (paxx extended firmware)

Needs SSH (`[remote_access] ssh: true`) and system persistence
(`touch /oem/.debug`). An `/etc/init.d` script added after install never runs
at boot on the U1, because `rcS` lists `S??*` before the overlay is mounted.
So Klipper starts the bridge instead, through a tiny extras module.

1. Copy `ace_bridge.py` to `/home/lava/printer_data/ace-bridge/` (owner `lava`).
2. Copy `klippy/ace_bridge.py` to `/home/lava/klipper/klippy/extras/ace_bridge.py`.
3. Copy `ace_bridge.cfg` to `config/extended/klipper/ace_bridge.cfg`. It sorts
   before `multiace.cfg`, so `[ace_bridge]` loads before `[ace]`.
4. Apply the multiACE option `v2_serial_paths`. It is on branch
   `v2-serial-paths` for current main. For v1.27b, use
   `multiace-v1.27b-v2_serial_paths.patch` in this folder.
5. In `config/extended/ace.cfg` `[ace]`, set:
   ```ini
   ace_device_count: 2
   v2_serial_paths: /tmp/ace-bridge/ace[0-9]*
   ```
6. Restart the Klipper **service**: `/etc/init.d/S60klipper restart`. A
   G-code `RESTART` keeps the old multiACE code loaded.

Never chain a second unit while plain multiACE (no bridge) has the port. Both
units answer address 0, multiACE sees slots flip empty and present, and it
starts insert routines on them.

## Tests

- `sim_chain.py` simulates a chain on a pty: units boot at address 0, the
  `ASSIGN` ack comes on the old address, addresses expire after 3.5 s, and
  broadcast replies are serialised. `SIGUSR1` "reboots" a unit.
- `test_client.py PORT_DIR [N]` plays multiACE on each virtual port. It checks
  that each port reaches its own unit (40 interleaved requests), that `ASSIGN`
  is blocked, and that addresses hold through 6 s of silence.

```
python3 sim_chain.py /tmp/sim.path &
python3 ace_bridge.py --bus "$(cat /tmp/sim.path)" --dir /tmp/t/ports --state /tmp/t/state.json --log /tmp/t/bridge.log &
python3 test_client.py /tmp/t/ports 2
```

## What is tested

On a Snapmaker U1 with two ACE 2 Pro on one CH343 cable: one on ACE2-Open
1.1.60O, the other first on stock 1.1.31 and then on 1.1.60O.

- `test_client.py` passes on the real hardware.
- `--identify` blinks only the addressed unit, checked in both directions.
- multiACE v1.27b plus `v2_serial_paths` detects both units, connected and
  ready, across a cold power-cycle.

**Not yet tested:** loads, retracts, tag reads, the dryer and feed assist
through the bridge, and three or four units.

## Credits

hakimio reverse-engineered the ACE 2 protocol. tobecwb measured the chain
behaviour and published the enumeration recipe (#120). Simon-CR built the
ACE2-Open firmware. decay71 wrote multiACE.

Licence: MIT. No affiliation with Anycubic.
