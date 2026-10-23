"""Protocol verification: round-trip, CRC rejection, resync, fragmentation."""
import os
import random
import struct
import sys

sys.path.insert(0, os.path.dirname(__file__))
import hil_protocol as P

ok = lambda c, m: print(("  PASS  " if c else "  FAIL  ") + m) or (c or sys.exit(1))

print("1. CRC known-answer")
# CRC-16/CCITT-FALSE of "123456789" is 0x29B1, the standard check value.
ok(P.crc16_ccitt(b"123456789") == 0x29B1,
   f"check value 0x29B1 -> 0x{P.crc16_ccitt(b'123456789'):04X}")

print("2. Payload round-trips")
s = P.pack_sensor(12.34, P.V_IMU | P.V_DVL | P.V_LOCK | P.V_DEPTH,
                  (0.1, -0.2, 0.3), (0.01, 0.02, 3.0), (0.5, 0.0, -0.1),
                  3.2, -8.5)
d = P.unpack_sensor(s)
ok(abs(d["t"] - 12.34) < 1e-12 and d["dvl_lock"] and abs(d["depth"] + 8.5) < 1e-12,
   f"sensor round-trip t={d['t']} lock={d['dvl_lock']}")

t = P.pack_thrust([1, 2, 3, 4, 5, 6], list(range(8)))
dt = P.unpack_thrust(t)
ok(dt["wrench"][0] == 1 and dt["thrusters"][7] == 7, "thrust round-trip")

st = P.pack_state([10, -8, -9], [0, 0, 3.1], 0.15, P.PHASE_ID["SEARCH"], 4200)
ds = P.unpack_state(st)
ok(ds["tick"] == 4200 and ds["phase_id"] == P.PHASE_ID["SEARCH"], "state round-trip")

pw = P.pack_pwm([1500] * 8)
ok(P.unpack_pwm(pw) == [1500] * 8, "pwm round-trip")

print("3. Frame encode/decode")
frame = P.encode(P.MSG_SENSOR, s)
dec = P.FrameDecoder()
got = dec.feed(frame)
ok(len(got) == 1 and got[0][0] == P.MSG_SENSOR and got[0][1] == s,
   "single frame decodes")

print("4. Byte-at-a-time fragmentation")
dec = P.FrameDecoder()
out = []
for byte in frame:
    out += dec.feed(bytes([byte]))
ok(len(out) == 1 and out[0][1] == s, "reassembled from single bytes")

print("5. Two frames back to back")
dec = P.FrameDecoder()
two = P.encode(P.MSG_ACK, b"") + P.encode(P.MSG_STATE, st)
got = dec.feed(two)
ok(len(got) == 2 and got[0][0] == P.MSG_ACK and got[1][0] == P.MSG_STATE,
   "two concatenated frames")

print("6. CRC corruption is rejected, not acted on")
dec = P.FrameDecoder()
bad = bytearray(P.encode(P.MSG_SENSOR, s))
bad[8] ^= 0xFF  # flip a payload byte
got = dec.feed(bytes(bad))
ok(len(got) == 0 and dec.crc_errors == 1, f"crc error caught ({dec.crc_errors})")

print("7. Resync after garbage")
dec = P.FrameDecoder()
noise = bytes(random.randint(0, 255) for _ in range(37))
got = dec.feed(noise + frame)
ok(len(got) == 1 and got[0][1] == s, f"resynced past {len(noise)} junk bytes")

print("8. Garbage that contains a false sync mid-stream")
dec = P.FrameDecoder()
tricky = bytes([P.SYNC0, P.SYNC1, 0x99]) + frame  # false start then real frame
got = dec.feed(tricky)
ok(any(g[1] == s for g in got), "recovers real frame after false sync")

print("9. Stress: 5000 random frames through a lossy channel")
dec = P.FrameDecoder()
rng = random.Random(0)
sent = 0
recv = 0
stream = bytearray()
expected = []
for _ in range(5000):
    mt = rng.choice([P.MSG_ACK, P.MSG_SENSOR, P.MSG_STATE])
    pl = {P.MSG_ACK: b"", P.MSG_SENSOR: s, P.MSG_STATE: st}[mt]
    stream += P.encode(mt, pl)
    expected.append((mt, pl))
    sent += 1
got = dec.feed(bytes(stream))
recv = len(got)
ok(recv == sent and got == expected,
   f"{recv}/{sent} frames intact, {dec.crc_errors} crc errors")

print("10. Pilot and safety messages, isonavi-rov")
pp = P.pack_pilot(12.5, (20.0, -3.5, 0, 0, 0, 0), -0.55, 0.031, P.F_MANUAL | P.F_KILL)
d = P.unpack_pilot(pp)
ok(d["t"] == 12.5 and d["tau"][1] == -3.5 and d["z_ref"] == -0.55 and d["hdg_err"] == 0.031,
   "pilot values round trip exactly")
ok(d["manual"] and d["kill"] and not d["leak"] and not d["done"], "pilot flags decode")
fr = P.encode(P.MSG_PILOT, pp)
got = P.FrameDecoder().feed(fr)
ok(len(got) == 1 and got[0] == (P.MSG_PILOT, pp), "pilot frame decodes")
sf = P.unpack_safety(P.pack_safety(3.0, "HOLD", 0.811))
ok(sf["state"] == "HOLD" and sf["scale"] == 0.811, "safety report round trips")

print()
print("protocol test complete")
