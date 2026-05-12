import time
from machine import Pin, I2C
from car_api import Motor, Ultrasonic, Servo, I2cLcd, Line_tracking

# --- Constants ---
DEFAULT_I2C_ADDR = 0x27
SPEED = 50
SPEED_LOW = 30

# Lookup table: sensor tuple -> (direction, speed, label)
LINE_ACTIONS = {
    (0, 1, 0): ("forward",       SPEED,     "Forward"),
    (1, 1, 0): ("left_forward",  SPEED,     "Left Forward"),
    (1, 0, 0): ("turn_left",     SPEED,     "Left"),
    (0, 1, 1): ("right_forward", SPEED,     "Right Forward"),
    (0, 0, 1): ("turn_right",    SPEED,     "Right"),
    (1, 1, 1): (None,            SPEED,     "Alignment Error"),   # None = stop
    (0, 0, 0): ("turn_right",    SPEED,     "Not Detected"),
}

# --- Setup ---
i2c = I2C(0, sda=Pin(20), scl=Pin(21), freq=400000)
lcd = I2cLcd(i2c, DEFAULT_I2C_ADDR, 2, 16)
line_tracking = Line_tracking()
motor = Motor()

# --- State ---
_last_sensor  = None
_last_label   = None

def _update_lcd(label):
    lcd.clear()
    lcd.putstr("Line Tracking\n")
    lcd.putstr(label)

def line_track():
    global _last_sensor, _last_label

    sensor = tuple(line_tracking.get_ir_value())

    # Only act if sensor state has changed
    if sensor == _last_sensor:
        return

    _last_sensor = sensor
    print(sensor)

    action = LINE_ACTIONS.get(sensor)
    if action is None:
        # Unknown sensor combination — stop and flag it
        motor.move(0, "forward", SPEED)
        label = f"Unknown {list(sensor)}"
        if label != _last_label:
            _update_lcd(label)
            _last_label = label
        return

    direction, spd, label = action
    if direction is None:
        motor.move(0, "forward", spd)   # stop
    else:
        motor.move(1, direction, spd)

    if label != _last_label:
        _update_lcd(label)
        _last_label = label

def line_track_stop():
    motor.move(0, "forward", SPEED)
    lcd.clear()

if __name__ == '__main__':
    try:
        while True:
            line_track()
    except KeyboardInterrupt:
        line_track_stop()