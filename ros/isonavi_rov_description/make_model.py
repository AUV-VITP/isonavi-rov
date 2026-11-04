"""Writes the Gazebo model of isonavi-rov from the solved budget and the CAD.

None of the vehicle's numbers in model.sdf is typed by hand. Mass, centre
of gravity, inertia, added mass and damping come from cad/rov_budget.json; the thruster stations
from cad/rov.py; the camera and lamps from the camera model the vision
results use; the visual meshes are exported from cad/rov_cad.py.

The link origin is the centre of gravity, as in the python dynamics, so the
damping, the tether moment and the odometry all refer to the same point.

Collisions do two jobs in Gazebo: contact, and the displaced volume the
buoyancy system integrates. The contact set is the parts a post or a wall
can actually meet, at their real sizes, with the side plates at the
thickness that leaves their budgeted volume after pocketing. The thrusters,
cables and the rest of the budgeted volume sit inside the frame, so one
non-contact volume carries them, placed so that the total volume and its
centre equal the budget's displaced volume and centre of buoyancy.

Run from the repository root with CadQuery available; it rewrites
models/isonavi_rov/.
"""

from __future__ import annotations

import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "cad"))
sys.path.insert(0, os.path.join(ROOT, "simulation"))
import rov as R                                      # noqa: E402
import rov_cad as C                                  # noqa: E402
from isonavi.dynamics import vectored_allocation     # noqa: E402
from isonavi.optics import CameraConfig              # noqa: E402
from isonavi.sensors import IMUConfig                # noqa: E402

OUT = os.path.join(HERE, "models", "isonavi_rov")
THRUSTER_LINK_KG = 0.05      # propeller links the thruster plugin spins; taken out of the base link
THRUSTER_CT = 0.40 / (2 * math.pi) ** 2     # cad/thruster.py's coefficient, per rad/s rather than rev/s


def f(v):
    return f"{v + 0.0:.6g}"          # + 0.0 turns -0.0 into 0


def vec(*v):
    return " ".join(f(x) for x in v)


class Model:
    def __init__(self):
        self.b = json.load(open(os.path.join(ROOT, "cad", "rov_budget.json")))
        self.cg = np.array([self.b["cg_m"][0], 0.0, self.b["cg_m"][1]])
        self.cb = np.array([self.b["cb_m"][0], 0.0, self.b["cb_m"][1]])

    def rel(self, p):
        """A frame position, relative to the link origin at the CG."""
        return np.asarray(p, float) - self.cg

    # ------------------------------------------------------------ collisions
    def contact_shapes(self):
        """(name, kind, size, frame position, rpy), at the sizes rov_cad.py builds."""
        b, out = self.b, []
        L, B, H = R.FRAME_L, R.FRAME_B, R.FRAME_H
        t = C.PLATE_T * (1 - C.POCKET_FRACTION)
        for s, side in ((1, "port"), (-1, "stbd")):
            out.append((f"side_plate_{side}", "box", (L, t, H), (0, s * (B / 2 - t / 2), 0), (0, 0, 0)))
        yi = B / 2 - C.PLATE_T
        for i, (x, z) in enumerate(((0.22, 0.12), (-0.22, 0.12), (0.22, -0.13), (-0.22, -0.13))):
            out.append((f"cross_member_{i + 1}", "box", (0.040, 2 * yi, 0.010), (x, 0, z), (0, 0, 0)))
        for s, side in ((1, "port"), (-1, "stbd")):
            out.append((f"skid_{side}", "box", (L * 0.95, 0.020, 0.020), (0, s * 0.15, -0.155), (0, 0, 0)))
        along_x = (0, math.pi / 2, 0)
        out.append(("hull", "cylinder", (C.HULL_OD / 2, C.HULL_L), (C.HULL_X, 0, C.HULL_Z), along_x))
        for s, end in ((-1, "rear"), (1, "front")):
            x = C.HULL_X + s * (C.HULL_L / 2 + C.CAP_T / 2)
            out.append((f"end_cap_{end}", "cylinder", (C.CAP_D / 2, C.CAP_T), (x, 0, C.HULL_Z), along_x))
        foam = next(p for p in b["parts"] if "foam" in p["name"])
        lf = foam["vol"] / (C.FOAM_W * C.FOAM_H)
        out.append(("foam", "box", (lf, C.FOAM_W, C.FOAM_H), (foam["x"], 0, C.FOAM_Z), (0, 0, 0)))
        lead = next(p for p in b["parts"] if "lead" in p["name"])
        out.append(("ballast", "box", (0.150, 0.060, lead["vol"] / (0.150 * 0.060)), (lead["x"], 0, -0.150),
                    (0, 0, 0)))
        for s, side in ((1, "port"), (-1, "stbd")):
            out.append((f"lamp_{side}", "cylinder", (C.LAMP_D / 2, C.LAMP_L), (C.LAMP_X, s * C.LAMP_Y, C.LAMP_Z),
                        along_x))
        probe_l = R.PROBE_TIP_X - 0.24
        out.append(("probe", "cylinder", (0.005, probe_l), (R.PROBE_TIP_X - probe_l / 2, 0, -0.06), along_x))
        out.append(("probe_bracket", "box", (0.020, 0.020, 0.059), (0.25, 0, -0.0945), (0, 0, 0)))
        out.append(("kill_switch", "cylinder", (0.015, 0.020), (-0.22, 0.15, 0.135), (0, 0, 0)))
        out.append(("tether_termination", "cylinder", (0.015, 0.040), (R.TETHER_END_X + 0.020, 0, R.TETHER_END_Z),
                    along_x))
        out.append(("tow_post", "box", (0.030, 0.010, 0.240), (-0.220, 0, -0.005), (0, 0, 0)))
        out.append(("tow_bracket", "box", (0.025, 0.020, 0.020), (-0.2475, 0, R.TETHER_END_Z), (0, 0, 0)))
        return out

    @staticmethod
    def volume(kind, size):
        if kind == "box":
            return size[0] * size[1] * size[2]
        return math.pi * size[0] ** 2 * size[1]

    def displacement(self):
        """Contact volume and the non-contact remainder that closes the budget."""
        shapes = self.contact_shapes()
        v = np.array([self.volume(k, s) for _, k, s, _, _ in shapes])
        c = np.array([p for _, _, _, p, _ in shapes], float)
        v_total = self.b["displaced_m3"]
        v_rest = v_total - v.sum()
        if v_rest <= 0:
            raise SystemExit(f"contact shapes displace {v.sum() * 1000:.2f} L, more than the budget's "
                             f"{v_total * 1000:.2f} L")
        c_rest = (v_total * self.cb - (v[:, None] * c).sum(axis=0)) / v_rest
        return shapes, v.sum(), v_rest, c_rest

    # -------------------------------------------------------------- thrusters
    def thrusters(self):
        """Frame position and rpy of each thruster, in vectored_allocation's order."""
        lx, ly, vx, vy = R.ARMS
        out = []
        for x, y, s in ((lx, ly, 1), (lx, -ly, -1), (-lx, ly, -1), (-lx, -ly, 1)):
            out.append(((x, y, R.H_THR_Z), (0, 0, s * math.pi / 4)))
        for x, y in ((vx, vy), (vx, -vy), (-vx, vy), (-vx, -vy)):
            out.append(((x, y, R.V_THR_Z), (0, -math.pi / 2, 0)))
        return out

    def check_thrusters(self):
        """The joint axes and stations reproduce the allocation matrix the controller uses."""
        k = self.b["dynamics"]
        B = vectored_allocation(R.ARMS, k["thruster_hz"], k["thruster_ox"])
        for j, (p, (_, pitch, yaw)) in enumerate(self.thrusters()):
            axis = np.array([math.cos(pitch) * math.cos(yaw), math.cos(pitch) * math.sin(yaw), -math.sin(pitch)])
            r = self.rel(p)
            col = np.concatenate([axis, np.cross(r, axis)])
            if not np.allclose(col, B[:, j], atol=1e-9):
                raise SystemExit(f"thruster {j + 1} does not match the allocation: {col} vs {B[:, j]}")

    # ----------------------------------------------------------------- inertia
    def base_inertial(self):
        """Budget mass and inertia, less the thruster links, so the model totals equal the budget."""
        k = self.b["dynamics"]
        m = self.b["mass_kg"] - 8 * THRUSTER_LINK_KG
        inertia = np.diag(k["inertia"]).astype(float)
        for p, _ in self.thrusters():
            r = self.rel(p)
            inertia -= THRUSTER_LINK_KG * (r @ r * np.eye(3) - np.outer(r, r))
        return m, inertia

    # -------------------------------------------------------------------- write
    def sdf(self, mesh_groups):
        k = self.b["dynamics"]
        m, inertia = self.base_inertial()
        shapes, v_contact, v_rest, c_rest = self.displacement()
        lin, quad, added = k["lin_damp"], k["quad_damp"], k["added_mass"]
        cam, imu = CameraConfig(), IMUConfig()
        x = []
        w = x.append
        w('<?xml version="1.0"?>')
        w("<!-- Generated by ros/isonavi_rov_description/make_model.py from cad/rov_budget.json. Do not edit. -->")
        w('<sdf version="1.11">')
        w('  <model name="isonavi_rov">')
        w('    <link name="base_link">')
        w("      <inertial>")
        w(f"        <mass>{f(m)}</mass>")
        w("        <inertia>")
        for n, (i, j) in (("ixx", (0, 0)), ("ixy", (0, 1)), ("ixz", (0, 2)), ("iyy", (1, 1)), ("iyz", (1, 2)),
                          ("izz", (2, 2))):
            w(f"          <{n}>{f(inertia[i, j])}</{n}>")
        w("        </inertia>")
        # Added mass is integrated implicitly by DART; the hydrodynamics plugin's
        # explicit version diverges when heave added mass exceeds the dry mass.
        w("        <fluid_added_mass>")
        for n, v in zip(("xx", "yy", "zz", "pp", "qq", "rr"), added):
            w(f"          <{n}>{f(v)}</{n}>")
        w("        </fluid_added_mass>")
        w("      </inertial>")
        for group, (rgba, mesh) in mesh_groups.items():
            w(f'      <visual name="{group}">')
            w(f"        <pose>{vec(*(-self.cg))} 0 0 0</pose>")
            w(f"        <geometry><mesh><uri>model://isonavi_rov/meshes/{mesh}</uri>"
              f"<scale>0.001 0.001 0.001</scale></mesh></geometry>")
            w(f"        <material><ambient>{vec(*rgba)}</ambient><diffuse>{vec(*rgba)}</diffuse></material>")
            w("      </visual>")
        for name, kind, size, p, rpy in shapes:
            w(f'      <collision name="{name}">')
            w(f"        <pose>{vec(*self.rel(p))} {vec(*rpy)}</pose>")
            if kind == "box":
                w(f"        <geometry><box><size>{vec(*size)}</size></box></geometry>")
            else:
                w(f"        <geometry><cylinder><radius>{f(size[0])}</radius><length>{f(size[1])}</length>"
                  "</cylinder></geometry>")
            w("      </collision>")
        r_rest = (3 * v_rest / (4 * math.pi)) ** (1 / 3)
        w('      <collision name="displacement">')
        w(f"        <pose>{vec(*self.rel(c_rest))} 0 0 0</pose>")
        w(f"        <geometry><sphere><radius>{f(r_rest)}</radius></sphere></geometry>")
        w("        <surface><contact><collide_bitmask>0x0000</collide_bitmask></contact></surface>")
        w("      </collision>")
        # Sensors and lamps where the camera model puts them, relative to the CG.
        w('      <sensor name="camera" type="camera">')
        w(f"        <pose>{vec(*cam.pos)} 0 0 0</pose>")
        w("        <always_on>true</always_on>")
        w("        <update_rate>10</update_rate>")
        w("        <topic>/model/isonavi_rov/camera/image</topic>")
        w("        <camera>")
        w(f"          <horizontal_fov>{f(math.radians(cam.hfov_deg))}</horizontal_fov>")
        w(f"          <image><width>{cam.width}</width><height>{cam.height}</height><format>R8G8B8</format></image>")
        w("          <clip><near>0.05</near><far>12</far></clip>")
        w(f"          <noise><type>gaussian</type><mean>0</mean><stddev>{f(cam.read_noise)}</stddev></noise>")
        w("        </camera>")
        w("      </sensor>")
        w('      <sensor name="imu" type="imu">')
        w("        <always_on>true</always_on>")
        w(f"        <update_rate>{f(imu.rate_hz)}</update_rate>")
        w("        <topic>/model/isonavi_rov/imu</topic>")
        w("        <imu>")
        for q, sd in (("angular_velocity", imu.gyro_sigma), ("linear_acceleration", imu.accel_sigma)):
            w(f"          <{q}>" + "".join(f'<{a}><noise type="gaussian"><stddev>{f(sd)}</stddev></noise></{a}>'
                                          for a in "xyz") + f"</{q}>")
        w("        </imu>")
        w("      </sensor>")
        half = math.degrees(math.acos(0.5 ** (1 / cam.lamps[0].beam_exp)))
        for lamp, side in zip(cam.lamps, ("port", "stbd")):
            p = np.asarray(cam.pos) + np.asarray(lamp.pos)
            w(f'      <light name="light_{side}" type="spot">')
            # A spot light shines along its own -z; pitch -90 degrees turns that forward.
            w(f"        <pose>{vec(*p)} 0 {f(-math.pi / 2)} 0</pose>")
            w("        <diffuse>1 0.97 0.9 1</diffuse><specular>0.2 0.2 0.2 1</specular>")
            w("        <attenuation><range>12</range><constant>0.2</constant><linear>0</linear>"
              "<quadratic>0.15</quadratic></attenuation>")
            # Full cone angles: the inner one at the cos^k beam's half power, the outer twice it.
            w(f"        <spot><inner_angle>{f(math.radians(2 * half))}</inner_angle>"
              f"<outer_angle>{f(math.radians(4 * half))}</outer_angle><falloff>1</falloff></spot>")
            w("        <cast_shadows>false</cast_shadows>")
            w("      </light>")
        w("    </link>")
        for j, (p, rpy) in enumerate(self.thrusters(), 1):
            w(f'    <link name="thruster_{j}">')
            w(f"      <pose>{vec(*self.rel(p))} {vec(*rpy)}</pose>")
            w(f"      <inertial><mass>{f(THRUSTER_LINK_KG)}</mass><inertia><ixx>1e-5</ixx><iyy>1e-5</iyy>"
              "<izz>1e-5</izz></inertia></inertial>")
            w("    </link>")
            w(f'    <joint name="thruster_{j}_joint" type="revolute">')
            w("      <parent>base_link</parent>")
            w(f"      <child>thruster_{j}</child>")
            # No joint damping: the plugin spins the propeller on a velocity command,
            # and damping on that joint would put its torque on the hull. The
            # python model leaves propeller torque out, as counter-rotating pairs do.
            w("      <axis><xyz>1 0 0</xyz><limit><effort>50</effort><velocity>600</velocity></limit></axis>")
            w("    </joint>")
            w('    <plugin filename="gz-sim-thruster-system" name="gz::sim::systems::Thruster">')
            w(f"      <joint_name>thruster_{j}_joint</joint_name>")
            w(f"      <thrust_coefficient>{f(THRUSTER_CT)}</thrust_coefficient>")
            w("      <fluid_density>1000</fluid_density>")
            w(f"      <propeller_diameter>{f(R.PROP_D)}</propeller_diameter>")
            # Spin the propeller by setting its joint speed. The default drives it with
            # a PID on joint torque, and that torque reacts on the hull and yaws it.
            w("      <velocity_control>true</velocity_control>")
            w(f"      <max_thrust_cmd>{f(R.THRUST_MAX_N)}</max_thrust_cmd>")
            w(f"      <min_thrust_cmd>{f(-R.THRUST_MAX_N)}</min_thrust_cmd>")
            w("    </plugin>")
        # Gazebo's coefficients are hydrodynamic derivatives, negative for damping.
        w('    <plugin filename="gz-sim-hydrodynamics-system" name="gz::sim::systems::Hydrodynamics">')
        w("      <link_name>base_link</link_name>")
        w("      <disable_added_mass>true</disable_added_mass>")
        for n, v in zip(("xU", "yV", "zW", "kP", "mQ", "nR"), lin):
            w(f"      <{n}>{f(-v)}</{n}>")
        for n, v in zip(("xUabsU", "yVabsV", "zWabsW", "kPabsP", "mQabsQ", "nRabsR"), quad):
            w(f"      <{n}>{f(-v)}</{n}>")
        w("    </plugin>")
        w("  </model>")
        w("</sdf>")
        return "\n".join(x) + "\n", v_contact, v_rest, c_rest

    def config(self):
        b = self.b
        return (f'<?xml version="1.0"?>\n<!-- Generated by make_model.py. Do not edit. -->\n<model>\n'
                f"  <name>isonavi_rov</name>\n  <version>1.0</version>\n  <sdf version=\"1.11\">model.sdf</sdf>\n"
                f"  <description>isonavi-rov from the solved budget: {b['mass_kg']:.2f} kg, "
                f"{b['displaced_m3'] * 1000:.2f} L displaced, {b['net_buoyancy_n']:+.2f} N net, "
                f"BG {b['bg_m'] * 1000:.0f} mm.</description>\n</model>\n")


def export_meshes(model):
    """One STL per colour from the CAD assembly, in millimetres; the SDF scales them."""
    import cadquery as cq
    assy, named, _ = C.build()
    groups = {}
    for name, (wp, group) in named.items():
        col = assy.objects[name].color
        rgba = tuple(round(float(c), 3) for c in col.toTuple()) if col is not None else (0.5, 0.5, 0.5, 1.0)
        groups.setdefault(rgba, []).append(C.solid_of(wp))
    os.makedirs(os.path.join(OUT, "meshes"), exist_ok=True)
    out = {}
    for i, (rgba, solids) in enumerate(sorted(groups.items())):
        mesh = f"part_{i}.stl"
        cq.exporters.export(cq.Compound.makeCompound(solids), os.path.join(OUT, "meshes", mesh),
                            tolerance=0.5, angularTolerance=0.3)
        out[f"visual_{i}"] = (rgba, mesh)
    return out


def main():
    model = Model()
    model.check_thrusters()
    meshes = export_meshes(model)
    sdf, v_contact, v_rest, c_rest = model.sdf(meshes)
    open(os.path.join(OUT, "model.sdf"), "w", newline="\n").write(sdf)
    open(os.path.join(OUT, "model.config"), "w", newline="\n").write(model.config())
    b = model.b
    print(f"model.sdf: {b['mass_kg']:.3f} kg with the thruster links, {b['displaced_m3'] * 1000:.2f} L displaced, "
          f"{v_contact * 1000:.2f} L in contact shapes and {v_rest * 1000:.2f} L carried at "
          f"x {c_rest[0]:+.3f} z {c_rest[2]:+.3f} m, {len(meshes)} visual meshes")


if __name__ == "__main__":
    main()
