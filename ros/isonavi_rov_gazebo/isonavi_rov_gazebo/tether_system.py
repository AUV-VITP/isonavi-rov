"""Tether drag in Gazebo, applied every physics step by the python system loader.

It calls the same tether_wrench the python course runs use, with the same
tether and the tow point from the solved budget, so the two cannot differ.
A ROS node cannot do this: Gazebo's persistent wrench topic adds each new
wrench to the ones before it rather than replacing them, so a drag that
changes every step has to be applied from inside the simulation.
"""

import numpy as np
from gz.math7 import Vector3d
from gz.sim8 import Link, Model

from isonavi.dynamics import isonavi_rov, rot_body_to_world
from isonavi.pilot import TetherConfig, tether_wrench


class TetherDrag:
    def configure(self, entity, sdf, ecm, event_mgr):
        self.link = Link(Model(entity).link_by_name(ecm, "base_link"))
        self.link.enable_velocity_checks(ecm, True)
        self.tc = TetherConfig()
        print("isonavi-rov tether drag: on base_link, tow point "
              + " ".join(f"{v:+.3f}" for v in isonavi_rov.tow_point) + " m from the CG", flush=True)

    def pre_update(self, info, ecm):
        if info.paused:
            return
        pose = self.link.world_pose(ecm)
        v = self.link.world_linear_velocity(ecm)
        if pose is None or v is None:
            return
        p, rpy = pose.pos(), pose.rot().euler()
        eta = np.array([p.x(), p.y(), p.z(), rpy.x(), rpy.y(), rpy.z()])
        R = rot_body_to_world(*eta[3:])
        nu = np.concatenate([R.T @ np.array([v.x(), v.y(), v.z()]), np.zeros(3)])
        w = tether_wrench(eta, nu, self.tc, isonavi_rov.tow_point)
        force, torque = R @ w[:3], R @ w[3:]
        self.link.add_world_wrench(ecm, Vector3d(*force), Vector3d(*torque))


def get_system():
    return TetherDrag()
