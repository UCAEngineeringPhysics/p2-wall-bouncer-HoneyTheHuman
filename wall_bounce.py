"""
Rush the robot to a wall, stop at the required distances.
Now extended to support:
- WORK / PAUSE modes (button)
- RGB LED status encoding
- Ultrasonic-based wall avoidance
- Low-battery simulation (time-based)
"""

# Right motor pins
# in1 = 15, in2 = 13, PWM = 14

# Left motor pins
# in1 = 10, in2 = 8, PWM = 9

# standby pin = 12

# sensor trigger pin = 3
# sensor echo pin = 2
# RGB LED pins = 21 (R), 22 (G), 26 (B)
# BUTTON_PIN is chosen as 20 here (change if your wiring is different)

from machine import Pin, PWM
from picozero import DistanceSensor
from dual_motor_driver import DualMotorDriver
from time import sleep, ticks_ms, ticks_diff

# =====================================================
# CONSTANTS / PINS
# =====================================================

BUTTON_PIN = 20            # update if your button is on a different GPIO
WORK_LOW_BATT_TIME = 45.0  # seconds
WORK_CRIT_BATT_TIME = 55.0 # seconds

# 1 Hz fade cycle in PAUSE (1 second total for fade-in + fade-out)
FADE_STEPS = 20           # number of steps in each fade half-cycle
FADE_FREQ = 1.0           # Hz

# =====================================================
# HARDWARE SETUP
# =====================================================

motor = DualMotorDriver(
    right_ids=(15, 13, 14),  # Right motor: IN1, IN2, PWM
    left_ids=(10, 8, 9),     # Left motor:  IN1, IN2, PWM
    stby_id=12
)
motor.stop()  # ensure motors are off at startup

sensor = DistanceSensor(trigger=3, echo=2)

# RGB LED using PWM for smooth fading
pwm_red = PWM(Pin(21, mode=Pin.OUT))
pwm_green = PWM(Pin(22, mode=Pin.OUT))
pwm_blue = PWM(Pin(26, mode=Pin.OUT))

for p in (pwm_red, pwm_green, pwm_blue):
    p.freq(1000)

# Button with pull-up: default should be 1 when not pressed
# Wiring: GPIO20 -> button -> GND
button = Pin(BUTTON_PIN, mode=Pin.IN, pull=Pin.PULL_UP)

print("program starting")

# =====================================================
# GLOBAL STATE
# =====================================================

mode = "PAUSE"        # "PAUSE" or "WORK"
running = True        # set to False when terminating
work_time = 0.0       # accumulated WORK MODE time in seconds

# =====================================================
# LED HELPERS
# =====================================================

def set_color(r_duty: int, g_duty: int, b_duty: int):
    """Set raw RGB brightness, 0..65535."""
    pwm_red.duty_u16(r_duty)
    pwm_green.duty_u16(g_duty)
    pwm_blue.duty_u16(b_duty)

def turn_off_leds():
    set_color(0, 0, 0)

def LED(color: str):
    """
    Named colors:
      'red', 'green', 'blue', 'on' (white-ish), 'off'
    """
    if color == "red":
        set_color(65535, 0, 0)
    elif color == "green":
        set_color(0, 65535, 0)
    elif color == "blue":
        set_color(0, 0, 65535)
    elif color == "on":  # all on
        set_color(65535, 65535, 65535)
    elif color == "off":
        set_color(0, 0, 0)

def get_mode_color() -> str:
    """
    Substitute GREEN LED with BLUE LED after 45 s of WORK time.
    Before 45 s: 'green'
    After 45 s: 'blue'
    """
    if work_time >= WORK_LOW_BATT_TIME:
        return "blue"
    else:
        return "green"

def set_mode_led_constant():
    """Constant LED for the current mode (WORK or PAUSE)."""
    color = get_mode_color()
    if color == "green":
        set_color(0, 65535, 0)
    else:
        set_color(0, 0, 65535)

def fade_mode_led_one_cycle():
    """
    Fade current mode color in and out at 1 Hz.
    Total cycle time: 1 second (0.5 up, 0.5 down).
    This is only used in PAUSE MODE.
    """
    global running, mode

    color = get_mode_color()
    # choose which channel to fade
    if color == "green":
        # fade green channel
        for i in range(FADE_STEPS + 1):
            duty = int(65535 * (i / FADE_STEPS))
            set_color(0, duty, 0)
            if check_button_for_mode_switch():
                return
            if not running:
                return
            sleep(0.5 / FADE_STEPS)
        for i in range(FADE_STEPS, -1, -1):
            duty = int(65535 * (i / FADE_STEPS))
            set_color(0, duty, 0)
            if check_button_for_mode_switch():
                return
            if not running:
                return
            sleep(0.5 / FADE_STEPS)
    else:
        # fade blue channel
        for i in range(FADE_STEPS + 1):
            duty = int(65535 * (i / FADE_STEPS))
            set_color(0, 0, duty)
            if check_button_for_mode_switch():
                return
            if not running:
                return
            sleep(0.5 / FADE_STEPS)
        for i in range(FADE_STEPS, -1, -1):
            duty = int(65535 * (i / FADE_STEPS))
            set_color(0, 0, duty)
            if check_button_for_mode_switch():
                return
            if not running:
                return
            sleep(0.5 / FADE_STEPS)

# =====================================================
# BUTTON HELPERS
# =====================================================

def button_is_pressed() -> bool:
    # PULL_UP wiring: 1 = released, 0 = pressed
    return button.value() == 0

def wait_for_button_release():
    while button_is_pressed():
        sleep(0.02)

def check_button_for_mode_switch() -> bool:
    """
    If button pressed, toggle mode immediately.
    Return True if switched.
    """
    global mode
    if button_is_pressed():
        wait_for_button_release()
        if mode == "PAUSE":
            mode = "WORK"
            print("Button pressed: switching to WORK MODE")
        else:
            mode = "PAUSE"
            print("Button pressed: switching to PAUSE MODE")
        return True
    return False

# =====================================================
# DISTANCE / RANGE HELPERS
# =====================================================

def read_distance_avg(samples=3):
    """Average a few distance readings to smooth noise."""
    total = 0.0
    count = 0
    for _ in range(samples):
        d = sensor.distance
        if d is not None:
            total += d
            count += 1
        sleep(0.01)
    if count == 0:
        return None
    return total / count

# =====================================================
# MOTOR HELPERS
# =====================================================

def get_speed() -> float:
    """
    Base speed is 0.5.
    After 45 s of WORK time, robot speed is 50% of original (0.25).
    """
    base_speed = 0.5
    if work_time >= WORK_LOW_BATT_TIME:
        return base_speed * 0.5
    else:
        return base_speed

def drive_forward():
    motor.linear_forward(get_speed())

def drive_backwards(scale=1.0):
    motor.linear_backward(get_speed() * scale)

def drive_stop():
    motor.stop()

# =====================================================
# INITIALIZATION / SYSTEM CHECK
# =====================================================

def system_check():
    """
    (3%) Initialization (System Check):
      - Check button default signal (should be 1 for PULL_UP).
      - Check ultrasonic sensor > 0 (non-zero distance).
      - If both ok, blink all LEDs at 5 Hz for 2 seconds.
      - Enter PAUSE MODE after this step.
    """
    global mode

    motor.stop()  # motors off during check
    turn_off_leds()

    dist = read_distance_avg()
    button_default = button.value()

    print("System check: distance =", dist, "button default =", button_default)

    if (dist is not None) and (dist > 0.0) and (button_default == 1):
        print("System check passed: blinking all LEDs at 5 Hz for 2 s")
        # 5 Hz -> period 0.2 s -> 10 cycles in 2 seconds
        for _ in range(10):
            LED("on")
            sleep(0.1)
            LED("off")
            sleep(0.1)
    else:
        print("System check failed conditions, no blink. Check wiring.")

    # After system check, robot should enter PAUSE MODE
    mode = "PAUSE"
    print("Entering PAUSE MODE after system check")

# =====================================================
# WORK / PAUSE MODE LOGIC
# =====================================================

def update_low_battery_state():
    """Update LEDs/speed effects based on work_time."""
    # This function exists for clarity; we mostly use get_mode_color/get_speed
    # and just call set_mode_led_constant() where needed.
    pass

def distance_based_motion_step():
    """
    Simple wall-avoidance strategy for WORK MODE:
      - If distance > 0.35 m: move forward full speed.
      - If 0.20 m < distance <= 0.35 m: move forward slower.
      - If distance <= 0.20 m: back up briefly, then stop.
    """
    d = read_distance_avg()
    print("Distance:", d)

    if d is None:
        # sensor glitch, be safe
        drive_stop()
        return

    if d > 0.35:
        drive_forward()
    elif d > 0.20:
        # slow approach
        motor.linear_forward(get_speed() * 0.6)
    else:
        # too close, back up a bit
        drive_backwards(scale=0.5)
        sleep(0.2)
        drive_stop()

def pause_mode_loop():
    """
    (PAUSE MODE)
      - Robot stopped.
      - Mode LED (green or blue after 45 s) fades at 1 Hz.
      - Button press immediately switches to WORK MODE.
      - WORK time does not increase here.
      - Robot must not move.
    """
    global running, mode

    drive_stop()
    print("PAUSE MODE active")

    while running and mode == "PAUSE":
        # fade one cycle; button check occurs inside fade function
        fade_mode_led_one_cycle()
        # mode may change inside fade function
        if not running:
            break

def work_mode_loop():
    """
    (WORK MODE)
      - Mode LED (green normally, blue after 45 s) stays ON.
      - Button press immediately switches to PAUSE MODE.
      - Robot moves based on distance sensor (avoid hitting wall).
      - Accumulate WORK MODE time.
      - Low-battery behavior:
          * After 45 s: use BLUE instead of GREEN, 50% speed.
          * After 55 s: RED blinks at 10 Hz for 5 seconds, then shutdown.
    """
    global running, mode, work_time

    print("WORK MODE active")
    last_ms = ticks_ms()

    while running and mode == "WORK":
        # time update
        now = ticks_ms()
        dt = ticks_diff(now, last_ms) / 1000.0
        last_ms = now
        work_time += dt

        print("Accumulated WORK time:", work_time)

        # critical battery / termination condition
        if work_time >= WORK_CRIT_BATT_TIME:
            termination_sequence()
            running = False
            return

        # keep correct constant mode LED (green or blue)
        set_mode_led_constant()

        # move according to distance sensor
        distance_based_motion_step()

        # allow immediate mode switch
        if check_button_for_mode_switch():
            # when leaving WORK, stop motors to be safe
            drive_stop()
            return

        sleep(0.05)

# =====================================================
# TERMINATION SEQUENCE
# =====================================================

def termination_sequence():
    """
    (Termination)
      - If WORK time > 55 s, blink RED at 10 Hz for 5 seconds
        (BLUE and GREEN off).
      - Shutdown system after blinking.
    """
    global running

    print("Entering termination sequence (low battery critical).")

    drive_stop()
    # Ensure GREEN and BLUE are off
    set_color(0, 0, 0)

    # 10 Hz for 5 seconds -> period 0.1 s -> 50 cycles
    for _ in range(50):
        # RED on
        set_color(65535, 0, 0)
        sleep(0.05)
        # RED off
        set_color(0, 0, 0)
        sleep(0.05)

    # final shutdown state
    drive_stop()
    turn_off_leds()
    print("System shutdown complete.")
    running = False

# =====================================================
# MAIN
# =====================================================

# 1) Initialization / system check
system_check()

# 2) Start in PAUSE MODE
while running:
    if mode == "PAUSE":
        pause_mode_loop()
    elif mode == "WORK":
        work_mode_loop()

print("Program ended.")

#Ai was used for debugging and organziation
