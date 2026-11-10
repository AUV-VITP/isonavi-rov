# Hardware in the loop

The vehicle software on the flight computer, with the pool, the dynamics and
the pilot on a host.

## Roles

| Node | Hardware | Runs |
| --- | --- | --- |
| host | x86 | pool, dynamics, sensor models and the pilot (`host/rov_plant.py`) |
| flight computer | LicheeRV Nano, RISC-V rv64 at 750 MHz | hold loops, thrust allocation, power governor, supervisor (`board/rov_computer.py`) |
| actuator | ESP32-D0WD-V3 | one ESC pulse width channel per thruster (`esp32/`) |

Each tick the flight computer asks the host for the sensor sample, takes the
pilot packet that came down the tether (or notes that none did), runs the
hold loops, allocates thrust, applies the governor and the supervisor, drives
the ESP32 and reports the wrench the thrusters produce. It never sees the
course or the truth. The hold loops, allocation, governor and supervisor are
imported from `simulation/isonavi`, so a matching run is evidence about the
same code.

## Wire protocol

`common/hil_protocol.py`. Framed as `A5 5A | type | len | payload | crc16`,
CRC-16/CCITT-FALSE. The decoder resynchronises after corruption and rejects bad
frames rather than acting on them. 12 protocol tests in
`common/test_protocol.py`, including a 5000-frame lossy-channel stress test.

## Layout

- `common/` protocol and its tests
- `host/rov_plant.py` the host side: pool, dynamics, sensors, pilot
- `board/rov_computer.py` the flight computer; `board/links.py` its ESP32 and host links
- `esp32/` actuator firmware, bridge and checks
- `deploy_board.py` copies the board code onto the LicheeRV
- `test_rov_loopback.py` the full loop on localhost with no hardware: an
  assisted course against the pure simulation, then link loss and a kill

## Reproducing

```bash
python test_rov_loopback.py         # no hardware; writes results/rov_loopback.json
python deploy_board.py              # push the board code
```
