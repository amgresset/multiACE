# Klipper launcher for ace-bridge (several daisy-chained ACE 2 Pro units on
# one cable, each presented as its own virtual serial port).
#
# Install as klippy/extras/ace_bridge.py and add a section that is read
# BEFORE [ace] (e.g. config/extended/klipper/ace_bridge.cfg):
#
#   [ace_bridge]
#   #script: /home/lava/printer_data/ace-bridge/ace_bridge.py
#   #port_dir: /tmp/ace-bridge
#   #state: /home/lava/printer_data/config/ace-bridge.json
#   #startup_wait: 15
#   #args:
#
# On config load it starts the bridge as its own process (it survives Klipper
# restarts and is only started once) and waits until the virtual ports are
# ready, so multiACE finds them when it scans. Why Klipper starts it: on the
# U1 an /etc/init.d script added after install is never run at boot (rcS
# lists S??* before the overlay is mounted).
#
# Licence: MIT.

import logging
import os
import subprocess
import time


class AceBridgeLauncher:
    def __init__(self, config):
        self.script = config.get(
            'script', '/home/lava/printer_data/ace-bridge/ace_bridge.py')
        self.port_dir = config.get('port_dir', '/tmp/ace-bridge')
        self.state = config.get(
            'state', '/home/lava/printer_data/config/ace-bridge.json')
        self.wait = config.getfloat('startup_wait', 15., minval=0.)
        self.args = config.get('args', '').split()
        self.ready = os.path.join(self.port_dir, 'ready')
        if self._pid():
            logging.info('[ace_bridge] already running (pid %d)', self._pid())
            return
        if not os.path.isfile(self.script):
            raise config.error('[ace_bridge] script not found: %s'
                               % self.script)
        cmd = ['/usr/bin/python3', self.script, '--dir', self.port_dir,
               '--state', self.state] + self.args
        logging.info('[ace_bridge] starting: %s', ' '.join(cmd))
        subprocess.Popen(cmd, stdin=subprocess.DEVNULL,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                         close_fds=True, start_new_session=True)
        deadline = time.monotonic() + self.wait
        while time.monotonic() < deadline:
            if self._pid():
                logging.info('[ace_bridge] ready (pid %d), ports: %s',
                             self._pid(), ', '.join(sorted(
                                 p for p in os.listdir(self.port_dir)
                                 if p.startswith('ace'))))
                return
            time.sleep(0.2)
        logging.warning('[ace_bridge] not ready after %.0f s - see '
                        'ace-bridge.log', self.wait)

    def _pid(self):
        try:
            with open(self.ready) as f:
                pid = int(f.read().strip())
            os.kill(pid, 0)
            return pid
        except (OSError, ValueError):
            return 0

    def get_status(self, eventtime):
        return {'running': bool(self._pid()), 'pid': self._pid(),
                'port_dir': self.port_dir}


def load_config(config):
    return AceBridgeLauncher(config)
