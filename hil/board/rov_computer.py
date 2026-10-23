"""Flight computer for isonavi-rov, on the same board and links as isonavi-1.

Each tick it asks the host for the sensor sample, takes the pilot packet
that came down the tether (or notes that none did), runs the hold loops,
allocates thrust, applies the power governor and the supervisor, drives the
ESP32, and reports the wrench the thrusters actually produce. The hold loops,
the allocation, the governor and the supervisor are imported from the
simulation package, so a matching run is evidence about the same code.

The board never sees ground truth or the course. It sees what the vehicle
would: IMU, depth, and the pilot's command.
"""

from __future__ import annotations

import argparse
import json
import struct
import sys
import time

import numpy as np

sys.path.insert(0, "/root/hil")
sys.path.insert(0, "/root/hil/common")

import hil_protocol as P                                    # noqa: E402
from flight_computer import HostLink, SerialLink, thrust_to_pwm   # noqa: E402
from isonavi.dynamics import VehicleDynamics, isonavi_rov    # noqa: E402
from isonavi.pilot import AssistController                  # noqa: E402
from isonavi.safety import PowerGovernor, Supervisor        # noqa: E402


class RovComputer:
    def __init__(self, host, port, esp_dev=None, dt=0.02, comms_timeout=0.5):
        self.link = HostLink(host, port)
        self.esp = SerialLink(esp_dev)
        self.p = isonavi_rov
        self.dt = dt
        self.alloc = VehicleDynamics(isonavi_rov)      # allocation only; never integrated
        self.assist = AssistController(self.p.buoyancy - self.p.weight)
        self.gov = PowerGovernor()
        self.sup = Supervisor(comms_timeout)
        self.cmd = None
        self.kill = self.leak = False
        self.t = 0.0
        self.tick = 0
        self.timing, self.t_comp, self.states = [], [], []

    def recv_pilot(self, timeout=5.0):
        """The pilot packet for this tick, or None if the host marked it as lost."""
        self.link.sock.settimeout(timeout)
        while True:
            for k, (mt, pl) in enumerate(self.link._pending):
                if mt in (P.MSG_PILOT, P.MSG_ACK):
                    self.link._pending.pop(k)
                    return P.unpack_pilot(pl) if mt == P.MSG_PILOT else None
            data = self.link.sock.recv(4096)
            if not data:
                raise ConnectionError("host closed")
            self.link._pending += self.link.dec.feed(data)

    def step(self, s, pilot):
        if pilot is not None:
            self.sup.pilot_packet(self.t)
            self.cmd = pilot
            self.kill = self.kill or pilot["kill"]
            self.leak = self.leak or pilot["leak"]
        state = self.sup.update(self.t, kill=self.kill, leak=self.leak)
        tau = np.zeros(6)
        if self.cmd is not None:
            tau = np.array(self.cmd["tau"], float)
            if not self.cmd["manual"]:
                tau[2:] = self.assist.wrench(self.cmd["z_ref"], s["depth"], np.array(s["att"]),
                                             np.array(s["gyro"]), self.cmd["hdg_err"], self.dt)[2:]
        f = self.alloc.allocate(tau)
        f, scale = self.gov.limit(f)
        f = self.sup.gate(f)
        wrench = self.alloc.B @ f
        return wrench, f, state, scale

    def run(self, max_time):
        self.link.send(P.MSG_HELLO, struct.pack("<if", 2, 1.0 / self.dt))
        while self.t < max_time:
            t0 = time.perf_counter()
            self.link.send(P.MSG_SENSOR_REQ, P.pack_sensor_req(self.t))
            s = P.unpack_sensor(self.link.recv_frame(P.MSG_SENSOR))
            pilot = self.recv_pilot()
            if pilot is not None and pilot["done"]:
                break
            t1 = time.perf_counter()
            wrench, f, state, scale = self.step(s, pilot)
            self.esp.send_pwm([thrust_to_pwm(fi, self.p.max_thrust_n) for fi in f])
            self.esp.poll_echo()
            self.link.send(P.MSG_THRUST, P.pack_thrust(wrench.tolist(), f.tolist()))
            self.link.send(P.MSG_SAFETY, P.pack_safety(self.t, state, scale))
            self.states.append(state)
            self.t += self.dt
            self.tick += 1
            now = time.perf_counter()
            self.timing.append((now - t0) * 1000.0)
            self.t_comp.append((now - t1) * 1000.0)
        return {
            "ticks": self.tick, "sim_time": self.t,
            "compute_ms_mean": float(np.mean(self.t_comp)) if self.t_comp else 0.0,
            "compute_ms_p99": float(np.percentile(self.t_comp, 99)) if self.t_comp else 0.0,
            "loop_ms_mean": float(np.mean(self.timing)) if self.timing else 0.0,
            "budget_ms": self.dt * 1000.0,
            "host_crc_errors": self.link.dec.crc_errors,
            "esp_link": self.esp.kind, "esp_pwm_sent": self.esp.sent,
            "esp_mismatches": self.esp.mismatches,
            "states": {k: self.states.count(k) for k in P.SAFETY_STATES},
        }


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="10.133.84.100")
    ap.add_argument("--port", type=int, default=5556)
    ap.add_argument("--esp", default="/dev/ttyS1")
    ap.add_argument("--max-time", type=float, default=300.0)
    args = ap.parse_args()
    rep = RovComputer(args.host, args.port, args.esp).run(args.max_time)
    print("REPORT " + json.dumps(rep))
