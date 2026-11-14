"""Small external-clock Veins IEEE 802.11p adapter for the V2I experiment."""
import os
from pathlib import Path
import subprocess
import sys
import time

from vanet.cobridge.omnet_connection_management.connection import OmnetConnection
from vanet.project_paths import PROJECT_DIRECTORY


class V2IRadio:
    def __init__(self, output, network_file, rsu=(140., -20., 6.), seed=575, ui='Cmdenv'):
        self.output = Path(output).resolve()
        self.output.mkdir(parents=True, exist_ok=True)
        self.socket = self.output / 'radio.sock'
        self.log = (self.output / 'omnet.log').open('w')
        env = dict(os.environ)
        env.update(OMNETPP_ROOT='/home/wht/v2x/omnetpp-6.1',
                   VEINS_ROOT='/home/wht/v2x/veins-5.3.1',
                   SUMO_HOME='/home/wht/sumo-1.22.0',
                   CPATH='/home/wht/sumo-1.22.0/src/foreign')
        cmd = [sys.executable, '-m', 'vanet.omnet.build_and_launch.run_network',
               '--ui', ui, '--socket', str(self.socket), '--results', str(self.output/'omnet'),
               '--seed', str(seed), '--network-file', str(Path(network_file).resolve()),
               '--map-name', 'V2IThreeLights', '--rsu-pose', *map(str, rsu)]
        self.process = subprocess.Popen(cmd, cwd=PROJECT_DIRECTORY, env=env,
                                        stdout=self.log, stderr=subprocess.STDOUT)
        self.connection = OmnetConnection(self.socket, timeout=20)
        self.step_id = 0
        self.sent = 0
        self.received = 0
        self.delays = []
        self._wait_start()

    def _wait_start(self):
        deadline = time.monotonic()+60
        while not self.socket.exists():
            if self.process.poll() is not None:
                raise RuntimeError('OMNeT++ exited: '+(self.output/'omnet.log').read_text()[-1800:])
            if time.monotonic()>deadline:
                raise TimeoutError('OMNeT++ startup timed out: '+(self.output/'omnet.log').read_text()[-900:])
            time.sleep(.02)
        self.connection.start()
        self._wait(lambda: self.connection.ready)

    def _wait(self, ready):
        deadline=time.monotonic()+20
        while not ready():
            self.connection.poll()
            if self.connection.error:
                raise RuntimeError(self.connection.error)
            if self.process.poll() is not None:
                raise RuntimeError('OMNeT++ exited: '+(self.output/'omnet.log').read_text()[-1200:])
            if time.monotonic()>deadline:
                raise TimeoutError('OMNeT++ radio step timed out')
            time.sleep(.001)

    def advance(self, nodes, payload=None, recipient='lincoln0'):
        until=round(self.connection.network_time+.05, 9)
        transmissions=[] if payload is None else [dict(sender='rsu0',recipient=recipient,payload=payload)]
        self.connection.submit(self.step_id,until,nodes,transmissions)
        self.sent+=len(transmissions)
        self._wait(lambda:self.connection.pending is None)
        result=self.connection.take_result()
        self.step_id+=1
        delivered=[d for d in result['deliveries'] if d['recipient']==recipient]
        self.received+=len(delivered)
        self.delays.extend(d['received_at']-d['sent_at'] for d in delivered)
        return delivered

    def close(self):
        try:
            if self.connection.ready and not self.connection.pending:
                self.connection.request_shutdown()
                self._wait(lambda: bool(self.connection.results))
        except (RuntimeError,TimeoutError):
            pass
        self.connection.close()
        if self.process.poll() is None:
            self.process.terminate()
            try:self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:self.process.kill();self.process.wait()
        self.log.close()

    def __enter__(self): return self
    def __exit__(self,*_): self.close()
