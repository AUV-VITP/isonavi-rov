# Actuator interface

The ESP32 that turns the flight computer's thrust allocation into real ESC
pulse widths, one channel per thruster: six on the build, with two spare.

## What it does

`actuator.py` runs on the ESP32 under MicroPython. It speaks the same framed,
CRC-16/CCITT checked protocol as the rest of the bench, so the widths it
receives are the ones the allocation produced, in the same wire format, not a
second friendlier one written for a demo.

Eight LEDC channels at the standard 50 Hz ESC frame, 1000 to 2000 us. On each
command it reports the duty registers back, so the echo is what the hardware is
emitting rather than what it was told to emit.

## Measured

| | |
| --- | --- |
| channels | 8 at 50 Hz |
| resolution | 0.305 us per LSB, 16 bit duty |
| commanded against measured | exact, 0 us across the full range |
| command to echo | 31 ms mean, 24 to 40 ms |
| pipeline lag | 1.0 frames standalone, 2.6 in the closed loop |
| cost to the control loop | 3.3 ms mean, 3.8 ms p99, measured on the board |

In a full closed loop run the flight computer commanded 1201 frames, 1198
came back, and every one of them exactly reproduced a width that had actually
been commanded. Zero CRC errors.

## Four bugs worth recording

Each of these produced a plausible looking wrong answer, which is the kind
worth writing down.

**`os.dupterm(None, 1)` is wrong on ESP32.** The console slot is 0; index 1
raises ValueError. Re-opening `machine.UART(0)` to take the port is also wrong,
because the console driver already owns it and the second open fails with
ESP_ERR_INVALID_STATE. Neither is needed: while the loop runs, the REPL is not
parsing input, so the program already owns stdin. It only needs
`micropython.kbd_intr(-1)` so a 0x03 inside a binary frame is not read as an
interrupt.

**MicroPython's bytearray has no slice deletion.** The host decoder consumes
its buffer with `del buf[:n]`, which is CPython only and raises TypeError on
the device. The device rebuilds the buffer by slicing instead.

**The LEDC channels do not latch together.** They are created in sequence, so
their period boundaries are staggered. Polling for all eight to agree races the
transition and, on timeout, reports a mix of old and new values: some channels
exact, others thirty to forty microseconds out. It now reads the registers
before applying the next command, which costs nothing and is always a settled
reading.

**Paramiko keeps one `_tcp_handler` per transport.** Requesting a second
reverse port forward with a handler silently replaces the first, so the
plant's traffic was routed to the actuator bridge and the flight computer sat
waiting for sensor frames that were going to an ESP32. One router keyed on the
arrival port fixes it.

## Bench topology

On the vehicle this interface hangs off a UART pin pair. On the bench the ESP32
enumerates as a COM port on Windows while the flight computer is a USB network
device, and WSL cannot open the port without usbipd. So the port is published
on a socket by `esp_bridge.py`, and the board reaches it through a second SSH
reverse tunnel. The bytes are identical end to end; only the transport differs,
and that difference is why the loop timing carries a few milliseconds it would
not carry on the vehicle.

## Running it

```bash
python esp_bridge.py --port 5558 --dev COM10      # on the Windows host
python arm_actuator.py --port 5558                # start the loop, idempotent
python verify_actuator.py                         # direct, no bridge

# then, on the flight computer
python3 rov_computer.py --esp tcp:<windows-ip>:5558
```

Arming is deliberately not an autostart on boot. A loop that owns the console
from power-on removes the recovery path; this way a plain reset always lands
back at a REPL, and `esptool` recovers the board in any case.
