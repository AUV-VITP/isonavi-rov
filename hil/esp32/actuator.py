"""isonavi actuator interface, running on the ESP32.

Speaks the same framed, CRC checked protocol as the rest of the bench, so the
pulse widths this generates are the ones the flight computer's allocation
actually produced, carried over the same wire format rather than a second,
friendlier one written for the demo.

Eight LEDC channels at the standard 50 Hz ESC frame. On each command the duty
registers are read back and converted to microseconds, so the echo reports what
the hardware is really emitting rather than what it was told to emit. That
distinction is the point of putting a real part in the loop.

Console handling, which took two attempts to get right on this chip:

  * ``os.dupterm(None, 1)`` is wrong here. On ESP32 the console slot is 0, and
    index 1 raises ValueError.
  * Re-opening ``machine.UART(0)`` is also wrong: the console driver already
    owns UART0 and a second open fails with ESP_ERR_INVALID_STATE.

Neither is necessary. While this loop is running the REPL is not parsing input,
so the running program already owns stdin. The only thing in the way is that a
0x03 byte inside a binary frame would raise KeyboardInterrupt, which
``micropython.kbd_intr(-1)`` disables for the duration.
"""

import struct
import sys
import time

from machine import Pin, PWM

# Eight ESC outputs. Chosen to avoid the strapping pins (0, 2, 12, 15) and the
# input-only pins (34 to 39), so nothing here disturbs boot.
ESC_PINS = (13, 14, 16, 17, 18, 19, 21, 22)
FREQ = 50                       # standard ESC frame, 20 ms
PERIOD_US = 1000000 // FREQ
DUTY_BITS = 16
DUTY_MAX = (1 << DUTY_BITS) - 1

SYNC0, SYNC1 = 0xA5, 0x5A
MSG_PWM = 0x30
MSG_PWM_ECHO = 0x31

NEUTRAL_US = 1500
MIN_US, MAX_US = 1000, 2000


def crc16_ccitt(data, crc=0xFFFF):
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) \
                else (crc << 1) & 0xFFFF
    return crc


def encode(msg_type, payload):
    header = struct.pack("<BH", msg_type, len(payload))
    crc = crc16_ccitt(header + payload)
    return bytes([SYNC0, SYNC1]) + header + payload + struct.pack("<H", crc)


def us_to_duty(us):
    """Microseconds to a 16 bit duty word, rounded rather than truncated.

    One LSB is 20000/65535 = 0.305 us at this frame rate, so rounding keeps the
    round trip inside half a microsecond instead of biasing every channel low.
    """
    us = MIN_US if us < MIN_US else (MAX_US if us > MAX_US else us)
    return (us * DUTY_MAX + PERIOD_US // 2) // PERIOD_US


def duty_to_us(duty):
    return (duty * PERIOD_US + DUTY_MAX // 2) // DUTY_MAX


class Actuators:
    """Eight ESC channels, armed at neutral."""

    def __init__(self, settle_ms=300):
        self.ch = []
        for p in ESC_PINS:
            pwm = PWM(Pin(p), freq=FREQ)
            pwm.duty_u16(us_to_duty(NEUTRAL_US))
            self.ch.append(pwm)
        # The LEDC duty registers do not latch instantly. Reading them before
        # they do reports zero, which looks like a dead channel and is not.
        time.sleep_ms(settle_ms)

    def apply(self, widths):
        for pwm, w in zip(self.ch, widths):
            pwm.duty_u16(us_to_duty(w))

    def readback(self):
        """What the hardware is actually emitting, from the duty registers."""
        return [duty_to_us(pwm.duty_u16()) for pwm in self.ch]

    def readback_then_apply(self, widths):
        """Report what the hardware currently holds, then command the new value.

        Waiting a full PWM period after each command to confirm it does give a
        true reading, but it blocks the interface for 21 ms and, at a 50 ms
        command interval, that back pressure pushed the flight computer's loop
        past its budget. Reading first costs nothing: by the time the next
        command arrives the previous one has long since latched, so the value
        reported is still measured from the duty registers rather than assumed.
        The echo simply answers the previous command, which the commander
        already accounts for as pipeline lag.
        """
        measured = self.readback()
        self.apply(widths)
        return measured

    def safe(self):
        for pwm in self.ch:
            pwm.duty_u16(us_to_duty(NEUTRAL_US))


class Decoder:
    """Same resynchronising decoder as the host side, in miniature."""

    def __init__(self):
        self.buf = bytearray()
        self.crc_errors = 0
        self.resyncs = 0

    def feed(self, data):
        """Consume bytes, return complete frames.

        The host side of this decoder deletes consumed bytes with ``del
        buf[:n]``. MicroPython's bytearray does not implement slice deletion,
        so the same line raises TypeError on the device; the buffer is rebuilt
        by slicing instead. Frames here are 23 bytes, so the copy is free.
        """
        out = []
        self.buf.extend(data)
        while True:
            b = self.buf
            n = len(b)
            if n < 5:
                return out
            if not (b[0] == SYNC0 and b[1] == SYNC1):
                i = self._find_sync(1)
                if i < 0:
                    self.buf = bytearray(b[-1:]) if n else bytearray()
                    return out
                self.buf = bytearray(b[i:])
                self.resyncs += 1
                continue
            mt = b[2]
            ln = b[3] | (b[4] << 8)
            total = 5 + ln + 2
            if n < total:
                if self._find_sync(2) >= 0:
                    self.buf = bytearray(b[2:])
                    self.resyncs += 1
                    continue
                return out
            payload = bytes(b[5:5 + ln])
            crc_rx = b[5 + ln] | (b[6 + ln] << 8)
            if crc_rx != crc16_ccitt(bytes(b[2:5]) + payload):
                self.crc_errors += 1
                self.buf = bytearray(b[2:])
                self.resyncs += 1
                continue
            self.buf = bytearray(b[total:])
            out.append((mt, payload))

    def _find_sync(self, start):
        b = self.buf
        for i in range(start, len(b) - 1):
            if b[i] == SYNC0 and b[i + 1] == SYNC1:
                return i
        return -1


def run(idle_safe_ms=2000):
    """Service PWM frames on the console stream until interrupted."""
    import micropython
    import select

    act = Actuators()
    dec = Decoder()

    sin = sys.stdin.buffer
    sout = sys.stdout.buffer
    poller = select.poll()
    poller.register(sin, select.POLLIN)

    # A binary frame may contain 0x03; it must not mean interrupt.
    micropython.kbd_intr(-1)

    # Announce the loop is live, and report the armed state. This doubles as
    # proof that run() actually started, which a silent loop cannot give.
    sout.write(encode(MSG_PWM_ECHO, struct.pack("<8H", *act.readback())))

    frames = 0
    last = time.ticks_ms()
    try:
        while True:
            # One byte per poll. Batching reads behind a second poll(0) looked
            # tidier and silently read nothing on this port, which cost an
            # afternoon; this is the pattern that measurably works.
            if poller.poll(5):
                b = sin.read(1)
                if b:
                    for mt, pl in dec.feed(b):
                        if mt == MSG_PWM and len(pl) == 16:
                            meas = act.readback_then_apply(
                                struct.unpack("<8H", pl))
                            frames += 1
                            sout.write(encode(MSG_PWM_ECHO,
                                              struct.pack("<8H", *meas)))
                    last = time.ticks_ms()
            elif time.ticks_diff(time.ticks_ms(), last) > idle_safe_ms:
                # Lost the commander: fail safe to neutral rather than holding
                # the last commanded thrust.
                act.safe()
                last = time.ticks_ms()
    finally:
        act.safe()
        micropython.kbd_intr(3)
        if frames:
            print("frames", frames)
    return frames
