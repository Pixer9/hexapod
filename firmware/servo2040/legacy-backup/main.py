# servo2040_main.py (MicroPython-safe: no typing generics)
import sys
import time
import uselect

try:
    import ujson as json
except Exception:
    import json

try:
    import uos as os
except Exception:
    os = None

try:
    from machine import Pin
except Exception:
    Pin = None

try:
    import neopixel
except Exception:
    neopixel = None

from servo import ServoCluster, servo2040

N_SERVOS = 18
STATE_FILE = "servo_state.json"

# --- Servo cluster ---
servos = ServoCluster(
    pio=0,
    sm=0,
    pins=list(range(servo2040.SERVO_1, servo2040.SERVO_18 + 1)),
)

def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v

# Neutral: coxa channels at 0, others at 90
neutral_angles = [90.0] * N_SERVOS
for i in range(N_SERVOS):
    if i % 3 == 0:
        neutral_angles[i] = 0.0

angles = neutral_angles[:]
armed = False

# --- Ramping state (non-blocking) ---
ramp_active = False
ramp_t0 = 0
ramp_ms = 0
ramp_from = None
ramp_to = None
last_apply_ms = 0
APPLY_HZ = 50  # how often to update PWM during ramp
APPLY_PERIOD_MS = int(1000 / APPLY_HZ)

# --- Optional servo power enable pin (best-effort) ---
servo_power_pin = None
servo_power_supported = False

def _find_servo_power_pin():
    # We don't know the exact symbol name; try common candidates safely.
    if Pin is None:
        return None
    candidates = [
        "SERVO_EN", "SERVO_ENABLE", "ENABLE", "EN",
        "SERVO_POWER_EN", "SERVO_PWR_EN", "POWER_EN",
        "VOUT_EN", "VOLTAGE_EN", "VSERV_EN",
        "PWR_EN", "PIN_SERVO_EN", "EN_SERVO",
    ]
    for name in candidates:
        if hasattr(servo2040, name):
            try:
                p = getattr(servo2040, name)
                return Pin(p, Pin.OUT)
            except Exception:
                pass
    return None

servo_power_pin = _find_servo_power_pin()
servo_power_supported = servo_power_pin is not None

# -----------------------------
# RGB / NeoPixel LED support
# -----------------------------
np = None
np_n = 0
rgb_supported = False

def _find_attr_first(obj, names):
    for n in names:
        if hasattr(obj, n):
            return getattr(obj, n)
    return None

def _init_rgb():
    global np, np_n, rgb_supported
    if neopixel is None or Pin is None:
        np = None
        rgb_supported = False
        return False

    # Try common symbol names used across different builds
    pin_no = _find_attr_first(servo2040, [
        "RGB_DATA", "NEOPIXEL", "WS2812", "LED_DATA", "RGB_PIN", "PIN_RGB"
    ])
    n_leds = _find_attr_first(servo2040, [
        "NUM_RGB_LEDS", "RGB_LEDS", "N_RGB_LEDS", "NUM_LEDS"
    ])

    if pin_no is None or n_leds is None:
        np = None
        rgb_supported = False
        return False

    try:
        np_n = int(n_leds)
        np = neopixel.NeoPixel(Pin(int(pin_no), Pin.OUT), np_n)
        rgb_supported = True
        rgb_fill(0, 0, 0)
        return True
    except Exception:
        np = None
        rgb_supported = False
        return False

def rgb_fill(r, g, b):
    if np is None:
        return
    r = int(clamp(r, 0, 255))
    g = int(clamp(g, 0, 255))
    b = int(clamp(b, 0, 255))
    for i in range(np_n):
        np[i] = (r, g, b)
    try:
        np.write()
    except Exception:
        pass

def set_armed_led_state(is_armed):
    # Armed: green, Disarmed: dim red
    if not rgb_supported:
        return
    if is_armed:
        rgb_fill(0, 64, 0)
    else:
        rgb_fill(32, 0, 0)

_init_rgb()
set_armed_led_state(False)

# -----------------------------
# Helpers / protocol
# -----------------------------
def write_line(msg):
    sys.stdout.write(msg + "\n")

def ok(msg=None):
    if msg:
        write_line("OK " + msg)
    else:
        write_line("OK")

def err(msg):
    write_line("ERR " + msg)

def apply_angles():
    # Apply all current commanded angles to PWM outputs.
    for i in range(N_SERVOS):
        servos.value(i, angles[i])

def load_state():
    global angles
    if os is None:
        return False
    try:
        with open(STATE_FILE, "r") as f:
            data = json.loads(f.read())
        a = data.get("angles", None)
        if a is None or len(a) != N_SERVOS:
            return False
        aa = []
        for v in a:
            try:
                aa.append(clamp(float(v), -90.0, 90.0))
            except Exception:
                return False
        angles = aa
        return True
    except Exception:
        return False

def save_state():
    if os is None:
        return False
    try:
        data = {"angles": angles}
        with open(STATE_FILE, "w") as f:
            f.write(json.dumps(data))
        return True
    except Exception:
        return False

def stop_ramp():
    global ramp_active, ramp_from, ramp_to, ramp_ms
    ramp_active = False
    ramp_from = None
    ramp_to = None
    ramp_ms = 0

def start_ramp_to(target_angles, duration_ms):
    global ramp_active, ramp_t0, ramp_ms, ramp_from, ramp_to, last_apply_ms
    duration_ms = int(max(0, duration_ms))

    if duration_ms == 0:
        stop_ramp()
        for i in range(N_SERVOS):
            angles[i] = target_angles[i]
        if armed:
            apply_angles()
        return

    ramp_active = True
    ramp_t0 = time.ticks_ms()
    ramp_ms = duration_ms
    ramp_from = angles[:]
    ramp_to = target_angles[:]
    last_apply_ms = 0  # force immediate apply on next tick

def tick_ramp():
    global last_apply_ms, ramp_active

    if not ramp_active:
        return

    now = time.ticks_ms()
    elapsed = time.ticks_diff(now, ramp_t0)

    if elapsed >= ramp_ms:
        for i in range(N_SERVOS):
            angles[i] = ramp_to[i]
        if armed:
            apply_angles()
        stop_ramp()
        return

    if last_apply_ms and time.ticks_diff(now, last_apply_ms) < APPLY_PERIOD_MS:
        return
    last_apply_ms = now

    t = elapsed / float(ramp_ms)
    for i in range(N_SERVOS):
        a0 = ramp_from[i]
        a1 = ramp_to[i]
        angles[i] = a0 + (a1 - a0) * t

    if armed:
        apply_angles()

# --- Command handlers ---

def handle_ping(parts):
    write_line("PONG")

def handle_get(parts):
    # GET -> status line
    a = " ".join([("{:.2f}".format(v)) for v in angles])
    ok("GET armed={} ramp={} {}".format(1 if armed else 0, 1 if ramp_active else 0, a))

def handle_pow(parts):
    # POW 0|1
    if len(parts) != 2:
        err("POW expects 1 arg (0 or 1)")
        return
    v = parts[1].strip()
    if not servo_power_supported:
        err("POW_UNSUPPORTED")
        return
    try:
        if v == "1":
            servo_power_pin.value(1)
            ok("POW 1")
            return
        if v == "0":
            servo_power_pin.value(0)
            ok("POW 0")
            return
        err("POW arg must be 0 or 1")
    except Exception:
        err("POW_FAIL")

def handle_led(parts):
    # LED 0|1   OR   LED R G B
    if not rgb_supported:
        err("LED_UNSUPPORTED")
        return

    if len(parts) == 2:
        v = parts[1].strip()
        if v == "0":
            rgb_fill(0, 0, 0)
            ok("LED 0")
            return
        if v == "1":
            set_armed_led_state(armed)
            ok("LED 1")
            return
        err("LED expects 0/1 or R G B")
        return

    if len(parts) == 4:
        try:
            r = int(parts[1]); g = int(parts[2]); b = int(parts[3])
        except Exception:
            err("LED RGB must be ints")
            return
        rgb_fill(r, g, b)
        ok("LED RGB")
        return

    err("LED expects: LED 0|1 or LED R G B")

def handle_arm(parts):
    global armed
    if len(parts) != 2:
        err("ARM expects 1 arg (0 or 1)")
        return

    v = parts[1].strip()

    if v == "1":
        armed = True

        # If we *can* enable servo power, do it before enabling PWM.
        if servo_power_supported:
            try:
                servo_power_pin.value(1)
            except Exception:
                pass

        servos.enable_all()
        time.sleep(0.05)
        apply_angles()

        # Update LED state
        set_armed_led_state(True)

        ok("ARM 1")
        return

    if v == "0":
        stop_ramp()
        armed = False
        servos.disable_all()

        # If we *can* cut servo power, do it here.
        if servo_power_supported:
            try:
                servo_power_pin.value(0)
            except Exception:
                pass

        # Update LED state
        set_armed_led_state(False)

        ok("ARM 0")
        return

    err("ARM arg must be 0 or 1")

def handle_neutral(parts):
    global angles
    stop_ramp()
    angles = neutral_angles[:]
    if armed:
        apply_angles()
    ok("NEUTRAL")

def handle_set(parts):
    global angles
    # SET A <18 floats>
    if len(parts) < 3:
        err("SET expects: SET A <18 vals>")
        return

    if not armed:
        err("NOT_ARMED")
        return

    mode = parts[1].upper()
    if mode != "A":
        err("Only angle mode supported: SET A ...")
        return

    vals = parts[2:]
    if len(vals) != N_SERVOS:
        err("SET A expects 18 values")
        return

    try:
        new_angles = [float(v) for v in vals]
    except Exception:
        err("SET A values must be numbers")
        return

    new_angles = [clamp(a, -90.0, 90.0) for a in new_angles]
    stop_ramp()
    angles = new_angles
    apply_angles()
    ok("SET")

def handle_setch(parts):
    global angles
    # SETCH A <ch> <angle>
    if len(parts) != 4:
        err("SETCH expects: SETCH A <ch> <angle>")
        return

    if not armed:
        err("NOT_ARMED")
        return

    mode = parts[1].upper()
    if mode != "A":
        err("Only angle mode supported: SETCH A ...")
        return

    try:
        ch = int(parts[2])
        val = float(parts[3])
    except Exception:
        err("Bad args")
        return

    if ch < 0 or ch >= N_SERVOS:
        err("Bad channel")
        return

    val = clamp(val, -90.0, 90.0)
    stop_ramp()
    angles[ch] = val
    servos.value(ch, val)
    ok("SETCH")

def handle_ramp(parts):
    # RAMP A <ms> <18 floats>
    if len(parts) < 4:
        err("RAMP expects: RAMP A <ms> <18 vals>")
        return

    if not armed:
        err("NOT_ARMED")
        return

    mode = parts[1].upper()
    if mode != "A":
        err("Only angle mode supported: RAMP A ...")
        return

    try:
        dur = int(float(parts[2]))
    except Exception:
        err("RAMP ms must be a number")
        return

    vals = parts[3:]
    if len(vals) != N_SERVOS:
        err("RAMP A expects 18 values")
        return

    try:
        target = [clamp(float(v), -90.0, 90.0) for v in vals]
    except Exception:
        err("RAMP A values must be numbers")
        return

    start_ramp_to(target, dur)
    ok("RAMP")

def handle_stop(parts):
    stop_ramp()
    ok("STOP")

def handle_save(parts):
    if save_state():
        ok("SAVE")
    else:
        err("SAVE_FAIL")

def handle_load(parts):
    global angles
    stop_ramp()
    if load_state():
        if armed:
            apply_angles()
        ok("LOAD")
    else:
        err("LOAD_FAIL")

def dispatch(line):
    s = line.strip()
    if not s:
        return

    s = s.replace(",", " ")
    parts = [p for p in s.split() if p]
    if not parts:
        return

    cmd = parts[0].upper()

    if cmd == "PING":
        handle_ping(parts)
    elif cmd == "GET":
        handle_get(parts)
    elif cmd == "POW":
        handle_pow(parts)
    elif cmd == "LED":
        handle_led(parts)
    elif cmd == "ARM":
        handle_arm(parts)
    elif cmd == "NEUTRAL":
        handle_neutral(parts)
    elif cmd == "SET":
        handle_set(parts)
    elif cmd == "SETCH":
        handle_setch(parts)
    elif cmd == "RAMP":
        handle_ramp(parts)
    elif cmd == "STOP":
        handle_stop(parts)
    elif cmd == "SAVE":
        handle_save(parts)
    elif cmd == "LOAD":
        handle_load(parts)
    else:
        err("UNKNOWN_CMD")

# --- IO loop ---
poll = uselect.poll()
poll.register(sys.stdin, uselect.POLLIN)

def readline_nonblocking():
    if poll.poll(0):
        try:
            return sys.stdin.readline()
        except Exception:
            return None
    return None

# Boot: try loading last commanded angles (optional)
loaded = load_state()

# Start fully disabled
servos.disable_all()
if servo_power_supported:
    try:
        servo_power_pin.value(0)
    except Exception:
        pass

# Set LED state at boot
set_armed_led_state(False)

write_line(
    "OK BOOT v2 SERVO state_loaded={} pow_supported={} led_supported={}".format(
        1 if loaded else 0,
        1 if servo_power_supported else 0,
        1 if rgb_supported else 0
    )
)

while True:
    # run ramp updates (even when no input)
    try:
        tick_ramp()
    except Exception:
        stop_ramp()

    line = readline_nonblocking()
    if line:
        try:
            dispatch(line)
        except Exception:
            err("EXC")
    else:
        time.sleep(0.002)
