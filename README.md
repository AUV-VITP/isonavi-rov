# isonavi-rov

isonavi as a tethered, piloted vehicle for confined water: a 10 kg, 60 cm
open frame ROV that threads a 1 m gate, finds and touches a coloured target in
an unlit pool by its own light, and flies a walled maze without touching it.
It keeps the isonavi engines (ray casting, 6-DOF dynamics, the eight thruster
vectored layout, the hardware-validated actuator path, the solved mass budget
and the generated bill of materials) and replaces what a pool does not need.

## What changed from isonavi, and why

| isonavi-1 | isonavi-rov | Reason |
| --- | --- | --- |
| Autonomous survey | Piloted over a 50 m tether, with assist and vision modes | A person flies it; the assist and vision loops are what cut contacts |
| 28 kg, 1074 mm faired hull, fins | 9.98 kg, 60 x 44 x 33 cm open frame | No current to fight; a small box clears a 1 m corridor |
| 50 m pressure rating, ring frames | One acrylic housing, 2 m water | Pool depth is under 2 m |
| Forward-looking sonar and DVL | Camera and two lamps, IMU, depth | The targets are identified by colour, in the dark |
| 1.5 kWh battery aboard | 48 V down the tether from a 230 V supply, no battery | Removes the battery and its leak risk from the vehicle |
| Mission state machine | Pilot assist, power governor, safety supervisor, ROS 2 | The vehicle must stop safely on a kill, a leak or a lost link |

The river survey stack (`acoustics.py`, `scene.py`, `mapping.py`, the
learned sonar perception and `docs/`) is unchanged and still runs. The
documents in `docs/` describe isonavi-1.

## Headline results

Pilot-in-the-loop course runs, 12 seeds per cell, in three water clarities
(scattering coefficient b). Every run in every cell finished the course,
passed the gate clean and touched the right target.

| Mode | Contacts, b = 0.05 /m | b = 0.4 /m | b = 1.0 /m | Time at b = 1.0 /m |
| --- | --- | --- | --- | --- |
| manual | 0.92 +/- 0.64 | 0.92 +/- 0.76 | 6.83 +/- 3.91 | 135.7 s |
| assist (depth, heading, attitude hold) | 0.42 +/- 0.49 | 0.33 +/- 0.47 | 5.42 +/- 4.35 | 130.6 s |
| auto (vision in the lateral loop) | 0.08 +/- 0.28 | 0.08 +/- 0.28 | 1.50 +/- 1.71 | 116.6 s |

| Vehicle | |
| --- | --- |
| Mass, envelope | 9.98 kg, 60 x 44 x 33 cm |
| Net buoyancy, BG, trim | +2.00 N, 45 mm, level |
| Surge force, all units / one lost | 84.9 N / 42.4 N |
| Speed, all units / one lost | 0.73 m/s / 0.50 m/s |
| Margin at 0.35 m/s with 8 m of tether trailing, one unit lost | 1.4 |
| Tether loop resistance | 0.344 ohm |
| Governed input, peak current, voltage aboard | 800 W, 19.4 A, 41.3 V |
| Surface supply, main fuse | 48 V 1000 W, 25 A |
| Unpowered rise from 1.6 m | 30 s |
| Target colour identified, lamps 0.2 m off axis | to 2.0 m at b = 0.05, 1.5 m at b = 0.4, 1.0 m at b = 1.0 |
| Cost, vehicle / with tether and surface station / total | ₹73,822 / ₹1,14,179 / ₹1,37,037 |

## What the simulation decided

**Lateral perception, not depth, is what costs contacts in murky water.**
Depth and heading hold halve contacts in clear water but barely help at
b = 1.0 /m, because the pilot's read of the corridor degrades with the two
way attenuation of the vehicle's own light. Closing the lateral loop on the
camera (`auto`) is the change that holds up: 1.5 contacts against 6.8.

**Colour goes before the target does.** Red is absorbed several times faster
than blue, and the lamps light the water in front of the lens. Undoing the
water using range from the target's known size beats a fixed threshold at the
edge of the envelope (0.75 against 0.50 at 2.5 m in clear water), but the
hard limit is signal against backscatter. The rule that follows: close to
1 m before committing to a colour.

**Lamps belong 0.2 m off the lens axis.** At b = 0.4 /m and 1.5 m, lamps
0.10 m off axis identify 69 % of targets; 0.20 m or more identify all of
them. The 44 cm beam has room for that.

**The tether is a third of the load.** At 0.35 m/s, 8 m of trailing tether
adds 9.4 N to the frame's 21.9 N. Sized against that with one unit lost, a
25 N thruster leaves almost no margin, which is what set the bench target at
30 N per unit.

**All eight thrusters at once is not a design point.** Uncapped they draw
1079 W, which would need a 40 A fuse. A governor in the allocator caps input
at 800 W and is exact in one step, because propeller power is homogeneous of
degree 3/2 in thrust; all eight together still make 24.3 N each.

## What is here

| Module | Purpose |
| --- | --- |
| `simulation/isonavi/pool.py` | Pool arena: sloping floor, gate, coloured targets, three section maze; contact model |
| `simulation/isonavi/optics.py` | Camera image formation under the vehicle's lamps: attenuation per channel, backscatter integrated per ray, sensor noise |
| `simulation/isonavi/vision.py` | Target finder: veil removal, blobs, range from size, attenuation correction, colour |
| `simulation/isonavi/pilot.py` | Operator model, assist controller, scored course run with tether drag |
| `simulation/isonavi/safety.py` | Power governor and the latching supervisor |
| `simulation/isonavi/dynamics.py` | isonavi-rov preset, read from the solved budget; external wrench input |
| `cad/rov.py` | Mass, buoyancy and trim solve, size check, thrust envelope with a unit lost, tether, fuse |
| `cad/thruster.py` | Motor KV matched to the propeller at the bench thrust point |
| `cad/rov_bom.py` | Bill of materials in rupees with the basis of every price and what is made in house |
| `ros/isonavi_rov_ros/` | ROS 2 nodes: pilot, safety, actuator, vision, and a launch file |
| `hil/` | Flight computer and ESP32 actuator interface, validated on hardware for isonavi |

## Safety chain

Four layers, each able to stop the thrusters alone: a fused mains supply with
an emergency stop; an external magnet kill switch that opens the thruster
rail; the supervisor, which zeroes thrust on a kill, a leak, or 0.5 s without
a pilot packet; and the ESP32, which drops every channel to neutral if the
flight computer goes quiet, which its firmware already does. The vehicle floats
positive, so zero thrust is the safe state. The ROS 2 chain was run end to end
under Jazzy: with the pilot link live the actuator node emits the expected
pulse widths, and when the joystick stops it reports `HOLD: pilot link lost`
and all eight channels go to 1500 us.

## Made in house

The eight thrusters (printed duct and propeller on a potted outrunner), the
end caps, the frame, the buoyancy, the lamp housings, the kill switch, the
tether floats and the thrust rig: 12 of 34 lines. The bench thrust target is
30 N per unit, checked on the rig before a thruster is fitted.

The motor is matched to the propeller rather than picked by habit
(`cad/thruster.py`). A 76 mm ducted propeller makes 30 N at about 2850 rpm,
so a 1000 KV winding would carry 27.4 A of phase current, past 80 % of its
30 A controller. The 750 KV winding carries 20.5 A and draws 114 W at the
bench point, and its motor and propeller efficiencies together reproduce
the 0.45 drive efficiency the power path was sized on.

## Running it

```bash
cd cad
python rov.py                                   # vehicle budget, envelope, tether, fuse
python thruster.py                              # motor and propeller match
python rov_bom.py                               # bill of materials

cd ../simulation
PYTHONPATH=. python tests/smoke_rov.py          # arena, camera, vision, pilot, safety
PYTHONPATH=. python eval/run_trials.py          # course runs by mode and water clarity
PYTHONPATH=. python eval/vision_eval.py         # identification by range, clarity, lamp placement
PYTHONPATH=. python eval/rov_figures.py         # figures

bash ../ros/test_chain.sh                       # ROS 2 build and chain test, under Jazzy
```

## Status

Vehicle budget, power path, pilot model, vision and safety chain are complete
and verified in simulation, and the ROS 2 layer runs end to end. The actuator
interface and the flight computer pattern carry over from isonavi, where they
were validated on hardware. The course geometry is parametric and set to
representative values; element sizes and spacing go in `ArenaConfig`. Next is
water: thruster bench curves, a hull vacuum and dunk test, trim in a tank, and
pilot hours on a mock course.
