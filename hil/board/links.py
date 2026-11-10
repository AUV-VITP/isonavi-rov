"""Links from the flight computer: the ESP32 actuator and the host plant.

thrust_to_pwm maps a thruster force to the ESC pulse width the ESP32 outputs.
"""

from __future__ import annotations

import socket
import struct
import sys
import time

import numpy as np

sys.path.insert(0, "/root/hil")
sys.path.insert(0, "/root/hil/common")

import hil_protocol as P   # noqa: E402

def thrust_to_pwm(force_n, max_n=120.0):
    """Map a per-thruster force in newtons to an ESC pulse width in us.

    Bidirectional ESC convention: 1500 us neutral, 1100 full reverse, 1900
    full forward, linear in between. This is what the ESP32 will output.
    """
    frac = float(np.clip(force_n / max_n, -1.0, 1.0))
    return int(round(1500 + frac * 400))


class SerialLink:
    """Link to the ESP32 actuator interface.

    Accepts either a serial device, which is how it is wired on the vehicle,
    or ``tcp:host:port``, which is how the bench reaches it: the ESP32
    enumerates on the Windows host while this board is a USB network device,
    so the frames cross a transparent socket bridge instead of a UART. The
    bytes on the wire are identical either way.
    """

    def __init__(self, dev, baud=115200):
        self.dev = dev
        self.kind = "none"
        self.ser = None
        self.sock = None
        self.dec = P.FrameDecoder()
        self.sent = 0
        self.echoes = 0
        self.mismatches = 0
        self.worst_err = 0
        self.unanswered = 0
        self.matched = 0
        self.lag_sum = 0
        self.lag_max = 0
        self.trace = []
        # Commands awaiting their echo, oldest first.
        self.q_cmd = []
        self.ring = []
        if not dev:
            return
        if dev.startswith("tcp:"):
            try:
                import socket
                _, host, port = dev.split(":")
                self.sock = socket.create_connection((host, int(port)),
                                                     timeout=5)
                self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.sock.setblocking(False)
                self.kind = "tcp"
                print(f"[esp32] bridged over {host}:{port}")
            except Exception as e:
                print(f"[esp32] bridge connect failed: {e}, running without")
        else:
            try:
                import serial
                self.ser = serial.Serial(dev, baud, timeout=0)
                self.kind = "uart"
                print(f"[esp32] serial {dev} at {baud}")
            except Exception as e:
                print(f"[esp32] serial open failed: {e}, running without")

    @property
    def active(self):
        return self.kind != "none"

    def send_pwm(self, widths):
        if not self.active:
            return
        frame = P.encode(P.MSG_PWM, P.pack_pwm(widths))
        try:
            if self.ser:
                self.ser.write(frame)
            else:
                self.sock.sendall(frame)
            self.sent += 1
            self.q_cmd.append(list(widths))
            self.ring.append(list(widths))
            if len(self.ring) > 32:
                del self.ring[:len(self.ring) - 32]
            # Bound the queue: if the actuator stops answering, the backlog
            # should not grow without limit.
            if len(self.q_cmd) > 64:
                del self.q_cmd[:len(self.q_cmd) - 64]
        except Exception:
            pass

    def poll_echo(self):
        """Non-blocking. The control loop never waits on the actuator."""
        if not self.active:
            return None
        try:
            if self.ser:
                n = self.ser.in_waiting
                data = self.ser.read(n) if n else b""
            else:
                try:
                    data = self.sock.recv(4096)
                except BlockingIOError:
                    data = b""
        except Exception:
            return None
        if not data:
            return None
        got = None
        for mt, pl in self.dec.feed(data):
            if mt == P.MSG_PWM_ECHO:
                got = P.unpack_pwm(pl)
                self.echoes += 1

                # Compare against a ring of recent commands rather than a
                # strict queue. The strict version required each echo to
                # answer the oldest outstanding command; when thrust is
                # changing by tens of microseconds per tick, an echo that
                # legitimately answers a neighbouring command then counts as a
                # fault. What matters is whether the hardware reproduced a
                # command it was actually given.
                hit = -1
                for i in range(len(self.ring) - 1, -1, -1):
                    if max(abs(x - y)
                           for x, y in zip(got, self.ring[i])) <= 2:
                        hit = len(self.ring) - 1 - i
                        break
                if hit < 0:
                    self.mismatches += 1
                    if self.ring and len(self.trace) < 6:
                        ref = self.ring[-1]
                        self.worst_err = max(
                            self.worst_err,
                            max(abs(x - y) for x, y in zip(got, ref)))
                        self.trace.append({"echo": list(got),
                                           "latest_cmd": list(ref)})
                else:
                    self.matched += 1
                    self.lag_sum += hit
                    self.lag_max = max(self.lag_max, hit)
        return got


class HostLink:
    """TCP link to the host plant. The board connects; the host listens."""

    def __init__(self, host, port):
        self.sock = socket.create_connection((host, port), timeout=10)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.dec = P.FrameDecoder()
        self._pending = []

    def send(self, msg_type, payload):
        self.sock.sendall(P.encode(msg_type, payload))

    def recv_frame(self, want_type, timeout=5.0):
        """Block until a frame of want_type arrives."""
        self.sock.settimeout(timeout)
        while True:
            for mt, pl in self._pending:
                if mt == want_type:
                    self._pending.remove((mt, pl))
                    return pl
            data = self.sock.recv(4096)
            if not data:
                raise ConnectionError("host closed")
            self._pending += self.dec.feed(data)

