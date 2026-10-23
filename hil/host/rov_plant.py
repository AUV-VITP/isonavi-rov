"""Host side of the isonavi-rov bench: the pool, the true dynamics, the sensors and the pilot.

The host is the world and the surface station. Each tick it samples the
sensors from the true state, runs the pilot model on what the pilot can see,
and sends both down the link; the board's wrench comes back and the host
integrates the plant and scores the run exactly as the pure simulation does.
The pilot, the plant and the scoring are the CourseRun methods; only the
vehicle side has moved to the board.

Faults are injected here, where a test can place them: a window in which no
pilot packet reaches the vehicle (an ACK goes in its place), and the kill and
leak inputs.
"""

from __future__ import annotations

import os
import socket
import struct
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import common.hil_protocol as P          # noqa: E402
from isonavi.pool import PoolArena        # noqa: E402
from isonavi.pilot import CourseRun       # noqa: E402


def serve(port, mode="assist", seed=0, max_time=420.0, drop=None, kill_at=None, ready=None):
    """Run one course against a board. drop is (t0, t1) with no pilot packets."""
    run = CourseRun(PoolArena(), mode=mode, seed=seed)
    run._reset_score()
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", port))
    srv.listen(1)
    if ready is not None:
        ready.set()
    conn, _ = srv.accept()
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    conn.settimeout(15.0)
    dec = P.FrameDecoder()
    pending = []

    def next_frame():
        while True:
            if pending:
                return pending.pop(0)
            data = conn.recv(4096)
            if not data:
                raise ConnectionError("board closed")
            pending.extend(dec.feed(data))

    t = 0.0
    finished = False
    pre = None
    trace = {"t": [], "z": [], "state": [], "state_t": [], "thrust_abs": [], "packet_t": []}
    t_wall = time.perf_counter()
    try:
        while t < max_time:
            mt, pl = next_frame()
            if mt == P.MSG_SENSOR_REQ:
                eta, nu = run.dyn.eta.copy(), run.dyn.nu.copy()
                while True:                      # a waypoint change re-reads, as the simulation does
                    gyro, att = run.imu.measure(eta, nu, run.dt)
                    z_meas = run.depth.measure(eta)
                    cmd = run.pilot(t, eta, att, gyro, z_meas)
                    if not isinstance(cmd, str):
                        break
                conn.sendall(P.encode(P.MSG_SENSOR, P.pack_sensor(
                    t, P.V_IMU | P.V_DEPTH, gyro, att, (0, 0, 0), 0.0, z_meas)))
                if cmd is None:
                    finished = True
                    conn.sendall(P.encode(P.MSG_PILOT, P.pack_pilot(t, [0] * 6, 0, 0, P.F_DONE)))
                    break
                flags = (P.F_MANUAL if cmd["manual"] else 0) | (P.F_KILL if kill_at and t >= kill_at else 0)
                if drop and drop[0] <= t < drop[1]:
                    conn.sendall(P.encode(P.MSG_ACK, b""))
                else:
                    conn.sendall(P.encode(P.MSG_PILOT, P.pack_pilot(
                        t, cmd["tau"], cmd["z_ref"], cmd["hdg_err"], flags)))
                    trace["packet_t"].append(t)
                pre = (eta, nu)
            elif mt == P.MSG_THRUST:
                d = P.unpack_thrust(pl)
                run.dyn.step(np.array(d["wrench"]), run.dt, tau_ext=run.external_wrench(*pre))
                t += run.dt
                run.score(t, run.dyn.eta)
                trace["t"].append(t)
                trace["z"].append(float(run.dyn.eta[2]))
                trace["thrust_abs"].append(float(np.abs(d["thrusters"]).sum()))
            elif mt == P.MSG_SAFETY:
                sf = P.unpack_safety(pl)
                trace["state"].append(sf["state"])
                trace["state_t"].append(sf["t"])
    except (ConnectionError, socket.timeout):
        pass
    wall = time.perf_counter() - t_wall
    conn.close()
    srv.close()
    res = run.result(t, finished)
    return res, trace, wall, dec.crc_errors
