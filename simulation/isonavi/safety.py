"""Safety chain for a tethered vehicle: power governor and supervisor.

Four independent layers, each able to stop the thrusters on its own:

  1. surface      a fused mains supply with an emergency stop on its output;
  2. vehicle      an external kill switch that opens the thruster power rail;
  3. supervisor   this module, on the flight computer: no pilot packet for
                  ``comms_timeout`` seconds, a leak, or the kill input, and
                  the thrust demand goes to zero and stays there;
  4. actuator     the ESP32 drops every channel to neutral if the flight
                  computer itself goes quiet, so a crashed computer cannot
                  leave a thruster running.
 
The vehicle floats positive, so zero thrust is a safe state: it rises.

The governor caps electrical input so the tether, fuse and supply can be
sized to a real number. Propeller power from momentum theory is homogeneous
of degree 3/2 in thrust, so scaling every thruster by k scales the total by
k^(3/2), and the k that meets the cap is exact in one step.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

RHO = 1000.0


@dataclass
class GovernorConfig:
    cap_w: float = 800.0         # input power, as sized in cad/rov.py
    hotel_w: float = 44.0
    prop_d: float = 0.076
    drive_eta: float = 0.45
    buck_eta: float = 0.94


class PowerGovernor:
    def __init__(self, cfg: GovernorConfig | None = None):
        self.cfg = cfg or GovernorConfig()
        a = math.pi * (self.cfg.prop_d / 2) ** 2
        self._k = 1.0 / (math.sqrt(2 * RHO * a) * self.cfg.drive_eta)

    def input_power(self, forces):
        f = np.abs(np.asarray(forces, float))
        return (self._k * float(np.sum(f ** 1.5)) + self.cfg.hotel_w) / self.cfg.buck_eta

    def limit(self, forces):
        """Priority-constrained thrust scaling under the electrical power cap.
        
        Priority 1 (Protected): Vertical thrusters (channels 4..7: heave, roll, pitch).
        Priority 2 (Derated): Horizontal thrusters (channels 0..3: surge, sway, yaw).
        Horizontal thrust takes the derating cut first to preserve attitude stability.
        """
        f = np.asarray(forces, float).copy()
        if len(f) != 8:
            mech = self._k * float(np.sum(np.abs(f) ** 1.5))
            budget = self.cfg.cap_w * self.cfg.buck_eta - self.cfg.hotel_w
            if mech <= budget or mech == 0.0:
                return f, 1.0
            k = (budget / mech) ** (2.0 / 3.0)
            return f * k, k

        f_horiz, f_vert = f[:4], f[4:]
        mech_vert = self._k * float(np.sum(np.abs(f_vert) ** 1.5))
        mech_horiz = self._k * float(np.sum(np.abs(f_horiz) ** 1.5))
        budget = self.cfg.cap_w * self.cfg.buck_eta - self.cfg.hotel_w

        if mech_vert + mech_horiz <= budget or (mech_vert + mech_horiz) == 0.0:
            return f, 1.0

        if mech_vert < budget:
            budget_horiz = max(0.0, budget - mech_vert)
            k_h = (budget_horiz / mech_horiz) ** (2.0 / 3.0) if mech_horiz > 0.0 else 0.0
            f[:4] = f_horiz * k_h
            return f, k_h
        else:
            # Under extreme vertical load, zero horizontal and scale vertical
            k_v = (budget / mech_vert) ** (2.0 / 3.0)
            f[:4] = 0.0
            f[4:] = f_vert * k_v
            return f, k_v


class Supervisor:
    """Latching safety state machine on the flight computer."""

    ARMED, HOLD, KILLED = "ARMED", "HOLD", "KILLED"

    def __init__(self, comms_timeout=0.5):
        self.timeout = comms_timeout
        self.state = self.ARMED
        self.last_packet = 0.0
        self.reason = ""

    def pilot_packet(self, t):
        self.last_packet = t
        if self.state == self.HOLD:
            self.state, self.reason = self.ARMED, ""

    def update(self, t, kill=False, leak=False):
        if self.state == self.KILLED:
            return self.state
        if kill or leak:
            self.state = self.KILLED
            self.reason = "kill switch" if kill else "leak"
        elif t - self.last_packet > self.timeout:
            self.state, self.reason = self.HOLD, "pilot link lost"
        return self.state

    def gate(self, forces):
        """Thrust actually sent to the actuator interface."""
        if self.state != self.ARMED:
            return np.zeros_like(np.asarray(forces, float))
        return np.asarray(forces, float)
