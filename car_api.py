from machine import Pin, PWM, I2C
import array, time
import rp2
import os
import machine
from time import sleep_ms
import micropython
import math
micropython.alloc_emergency_exception_buf(100)

Dir_forward = 1
Dir_backward = -1

LF_forward = 1
LF_backward = -1
LB_forward = 1
LB_backward = -1
RF_forward = 1
RF_backward = -1
RB_forward = 1
RB_backward = -1


# left front wheel M1
Motor_LF_PWM = PWM(Pin(12))
Motor_LF_Dir = Pin(13, Pin.OUT)
# right front wheel M2
Motor_RF_PWM = PWM(Pin(15))
Motor_RF_Dir = Pin(14, Pin.OUT)
# right back wheel M3
Motor_RB_PWM = PWM(Pin(16))
Motor_RB_Dir = Pin(17, Pin.OUT)
# left back wheel M4
Motor_LB_PWM = PWM(Pin(19))
Motor_LB_Dir = Pin(18, Pin.OUT)

# mapping function
def map(x,in_max, in_min, out_max, out_min):
    return (x - in_min)/(in_max - in_min)*(out_max - out_min) + out_min

class Motor:
    # Pin pairs: (pwm_forward_pin, dir_pin)
    MOTOR_PINS = {
        'LF': (12, 13),
        'RF': (15, 14),
        'RB': (17, 16),
        'LB': (18, 19),
    }

    # direction -> [(motor, forward?), ...]  True=forward, False=backward, None=stop
    MOVE_MAP = {
        'forward':        {'LF': (True,  1), 'RF': (True,  1), 'RB': (True,  1), 'LB': (True,  1)},
        'backward':       {'LF': (False, 1), 'RF': (False, 1), 'RB': (False, 1), 'LB': (False, 1)},
        'right':          {'LF': (True,  1), 'RF': (False, 1), 'RB': (True,  1), 'LB': (False, 1)},
        'left':           {'LF': (False, 1), 'RF': (True,  1), 'RB': (False, 1), 'LB': (True,  1)},
        'left_forward':   {'LF': (True, .5), 'RF': (True,  1), 'RB': (True,  1), 'LB': (True, .5)},
        'right_forward':  {'LF': (True,  1), 'RF': (True, .5), 'RB': (True, .5), 'LB': (True,  1)},
        'left_backward':  {'LF': (False, .5), 'RF': (False,  1), 'RB': (False,  1), 'LB': (False, .5)},
        'right_backward': {'LF': (False,  1), 'RF': (False, .5), 'RB': (False, .5), 'LB': (False,  1)},
        'turn_left':      {'LF': (False, 1), 'RF': (True,  1), 'RB': (True,  1), 'LB': (False, 1)},
        'turn_right':     {'LF': (True,  1), 'RF': (False, 1), 'RB': (False, 1), 'LB': (True,  1)},
    }

    def __init__(self):
        # Cache PWM and Dir pin objects once at init
        self._motors = {}
        for name, (pwm_pin, dir_pin) in self.MOTOR_PINS.items():
            self._motors[name] = {
                'pwm_fwd': PWM(Pin(pwm_pin)),
                'pwm_bwd': PWM(Pin(dir_pin)),
                'dir_fwd': Pin(dir_pin, Pin.OUT),
                'dir_bwd': Pin(pwm_pin, Pin.OUT),
            }
            self._motors[name]['pwm_fwd'].freq(500)
            self._motors[name]['pwm_bwd'].freq(500)

    def _set_motor(self, name, forward, speed):
        """Drive one motor. forward=True/False, speed=0-100."""
        m = self._motors[name]
        duty = int(speed / 100 * 65535)
        if forward:
            m['dir_fwd'].low()
            m['pwm_fwd'].duty_u16(duty)
            m['pwm_bwd'].duty_u16(0)
        else:
            m['dir_bwd'].low()
            m['pwm_bwd'].duty_u16(duty)
            m['pwm_fwd'].duty_u16(0)

    def motor_stop(self):
        for name in self._motors:
            m = self._motors[name]
            m['pwm_fwd'].duty_u16(0)
            m['pwm_bwd'].duty_u16(0)

    def move(self, status, direction, speed):
        if status == 0 or direction not in self.MOVE_MAP:
            if direction not in self.MOVE_MAP:
                print("Direction error!")
            self.motor_stop()
            return

        for motor_name, (fwd, scale) in self.MOVE_MAP[direction].items():
            if fwd is None:
                # Motor disabled for this direction
                m = self._motors[motor_name]
                m['pwm_fwd'].duty_u16(0)
                m['pwm_bwd'].duty_u16(0)
            else:
                self._set_motor(motor_name, fwd, speed * scale)
          
  
class Line_tracking():
    def __init__(self):
        self.ir_left = Pin(6, Pin.IN)
        self.ir_middle = Pin(5, Pin.IN)
        self.ir_right = Pin(4, Pin.IN)
        
    def get_ir_value(self):
        return [self.ir_left.value(),self.ir_middle.value() ,self.ir_right.value()]
    

class LcdApi:
    """Implements the API for talking with HD44780 compatible character LCDs.
    This class only knows what commands to send to the LCD, and not how to get
    them to the LCD.

    It is expected that a derived class will implement the hal_xxx functions.
    """
    # HD44780 LCD controller command set

    LCD_CLR = 0x01              # DB0: clear display
    LCD_HOME = 0x02             # DB1: return to home position

    LCD_ENTRY_MODE = 0x04       # DB2: set entry mode
    LCD_ENTRY_INC = 0x02        # --DB1: increment
    LCD_ENTRY_SHIFT = 0x01      # --DB0: shift

    LCD_ON_CTRL = 0x08          # DB3: turn lcd/cursor on
    LCD_ON_DISPLAY = 0x04       # --DB2: turn display on
    LCD_ON_CURSOR = 0x02        # --DB1: turn cursor on
    LCD_ON_BLINK = 0x01         # --DB0: blinking cursor

    LCD_MOVE = 0x10             # DB4: move cursor/display
    LCD_MOVE_DISP = 0x08        # --DB3: move display (0-> move cursor)
    LCD_MOVE_RIGHT = 0x04       # --DB2: move right (0-> left)

    LCD_FUNCTION = 0x20         # DB5: function set
    LCD_FUNCTION_8BIT = 0x10    # --DB4: set 8BIT mode (0->4BIT mode)
    LCD_FUNCTION_2LINES = 0x08  # --DB3: two lines (0->one line)
    LCD_FUNCTION_10DOTS = 0x04  # --DB2: 5x10 font (0->5x7 font)
    LCD_FUNCTION_RESET = 0x30   # See "Initializing by Instruction" section

    LCD_CGRAM = 0x40            # DB6: set CG RAM address
    LCD_DDRAM = 0x80            # DB7: set DD RAM address

    LCD_RS_CMD = 0
    LCD_RS_DATA = 1

    LCD_RW_WRITE = 0
    LCD_RW_READ = 1

    def __init__(self, num_lines, num_columns):
        self.num_lines = num_lines
        if self.num_lines > 4:
            self.num_lines = 4
        self.num_columns = num_columns
        if self.num_columns > 40:
            self.num_columns = 40
        self.cursor_x = 0
        self.cursor_y = 0
        self.backlight = True
        self.display_off()
        self.backlight_on()
        self.clear()
        self.hal_write_command(self.LCD_ENTRY_MODE | self.LCD_ENTRY_INC)
        self.hide_cursor()
        self.display_on()

    def clear(self):
        """Clears the LCD display and moves the cursor to the top left
        corner.
        """
        self.hal_write_command(self.LCD_CLR)
        self.hal_write_command(self.LCD_HOME)
        self.cursor_x = 0
        self.cursor_y = 0

    def show_cursor(self):
        """Causes the cursor to be made visible."""
        self.hal_write_command(self.LCD_ON_CTRL | self.LCD_ON_DISPLAY |
                               self.LCD_ON_CURSOR)

    def hide_cursor(self):
        """Causes the cursor to be hidden."""
        self.hal_write_command(self.LCD_ON_CTRL | self.LCD_ON_DISPLAY)

    def blink_cursor_on(self):
        """Turns on the cursor, and makes it blink."""
        self.hal_write_command(self.LCD_ON_CTRL | self.LCD_ON_DISPLAY |
                               self.LCD_ON_CURSOR | self.LCD_ON_BLINK)

    def blink_cursor_off(self):
        """Turns on the cursor, and makes it no blink (i.e. be solid)."""
        self.hal_write_command(self.LCD_ON_CTRL | self.LCD_ON_DISPLAY |
                               self.LCD_ON_CURSOR)

    def display_on(self):
        """Turns on (i.e. unblanks) the LCD."""
        self.hal_write_command(self.LCD_ON_CTRL | self.LCD_ON_DISPLAY)

    def display_off(self):
        """Turns off (i.e. blanks) the LCD."""
        self.hal_write_command(self.LCD_ON_CTRL)

    def backlight_on(self):
        """Turns the backlight on.

        This isn't really an LCD command, but some modules have backlight
        controls, so this allows the hal to pass through the command.
        """
        self.backlight = True
        self.hal_backlight_on()

    def backlight_off(self):
        """Turns the backlight off.

        This isn't really an LCD command, but some modules have backlight
        controls, so this allows the hal to pass through the command.
        """
        self.backlight = False
        self.hal_backlight_off()

    def move_to(self, cursor_x, cursor_y):
        """Moves the cursor position to the indicated position. The cursor
        position is zero based (i.e. cursor_x == 0 indicates first column).
        """
        self.cursor_x = cursor_x
        self.cursor_y = cursor_y
        addr = cursor_x & 0x3f
        if cursor_y & 1:
            addr += 0x40    # Lines 1 & 3 add 0x40
        if cursor_y & 2:
            addr += 0x14    # Lines 2 & 3 add 0x14
        self.hal_write_command(self.LCD_DDRAM | addr)

    def putchar(self, char):
        """Writes the indicated character to the LCD at the current cursor
        position, and advances the cursor by one position.
        """
        if char != '\n':
            self.hal_write_data(ord(char))
            self.cursor_x += 1
        if self.cursor_x >= self.num_columns or char == '\n':
            self.cursor_x = 0
            self.cursor_y += 1
            if self.cursor_y >= self.num_lines:
                self.cursor_y = 0
            self.move_to(self.cursor_x, self.cursor_y)

    def putstr(self, string):
        """Write the indicated string to the LCD at the current cursor
        position and advances the cursor position appropriately.
        """
        for char in string:
            self.putchar(char)

    def custom_char(self, location, charmap):
        """Write a character to one of the 8 CGRAM locations, available
        as chr(0) through chr(7).
        """
        location &= 0x7
        self.hal_write_command(self.LCD_CGRAM | (location << 3))
        time.sleep_us(40)
        for i in range(8):
            self.hal_write_data(charmap[i])
            time.sleep_us(40)
        self.move_to(self.cursor_x, self.cursor_y)

    def hal_backlight_on(self):
        """Allows the hal layer to turn the backlight on.

        If desired, a derived HAL class will implement this function.
        """
        pass

    def hal_backlight_off(self):
        """Allows the hal layer to turn the backlight off.

        If desired, a derived HAL class will implement this function.
        """
        pass

    def hal_write_command(self, cmd):
        """Write a command to the LCD.

        It is expected that a derived HAL class will implement this
        function.
        """
        raise NotImplementedError

    def hal_write_data(self, data):
        """Write data to the LCD.

        It is expected that a derived HAL class will implement this
        function.
        """
        raise NotImplementedError
    

DEFAULT_I2C_ADDR = 0x27


MASK_RS = 0x01
MASK_RW = 0x02
MASK_E = 0x04
SHIFT_BACKLIGHT = 3
SHIFT_DATA = 4


class I2cLcd(LcdApi):
    """Implements a HD44780 character LCD connected via PCF8574 on I2C."""

    def __init__(self, i2c, i2c_addr, num_lines, num_columns):
        self.i2c = i2c
        self.i2c_addr = i2c_addr
        self.i2c.writeto(self.i2c_addr, bytearray([0]))
        sleep_ms(20)   # Allow LCD time to powerup
        # Send reset 3 times
        self.hal_write_init_nibble(self.LCD_FUNCTION_RESET)
        sleep_ms(5)    # need to delay at least 4.1 msec
        self.hal_write_init_nibble(self.LCD_FUNCTION_RESET)
        sleep_ms(1)
        self.hal_write_init_nibble(self.LCD_FUNCTION_RESET)
        sleep_ms(1)
        # Put LCD into 4 bit mode
        self.hal_write_init_nibble(self.LCD_FUNCTION)
        sleep_ms(1)
        LcdApi.__init__(self, num_lines, num_columns)
        cmd = self.LCD_FUNCTION
        if num_lines > 1:
            cmd |= self.LCD_FUNCTION_2LINES
        self.hal_write_command(cmd)

    def hal_write_init_nibble(self, nibble):
        """Writes an initialization nibble to the LCD.

        This particular function is only used during initialization.
        """
        byte = ((nibble >> 4) & 0x0f) << SHIFT_DATA
        self.i2c.writeto(self.i2c_addr, bytearray([byte | MASK_E]))
        self.i2c.writeto(self.i2c_addr, bytearray([byte]))

    def hal_backlight_on(self):
        """Allows the hal layer to turn the backlight on."""
        self.i2c.writeto(self.i2c_addr, bytearray([1 << SHIFT_BACKLIGHT]))

    def hal_backlight_off(self):
        """Allows the hal layer to turn the backlight off."""
        self.i2c.writeto(self.i2c_addr, bytearray([0]))

    def hal_write_command(self, cmd):
        """Writes a command to the LCD.

        Data is latched(锁定) on the falling edge of E.
        """
        byte = ((self.backlight << SHIFT_BACKLIGHT) | (((cmd >> 4) & 0x0f) << SHIFT_DATA))
        self.i2c.writeto(self.i2c_addr, bytearray([byte | MASK_E]))
        self.i2c.writeto(self.i2c_addr, bytearray([byte]))
        byte = ((self.backlight << SHIFT_BACKLIGHT) | ((cmd & 0x0f) << SHIFT_DATA))
        self.i2c.writeto(self.i2c_addr, bytearray([byte | MASK_E]))
        self.i2c.writeto(self.i2c_addr, bytearray([byte]))
        if cmd <= 3:
            # The home and clear commands require a worst case delay of 4.1 msec
            sleep_ms(5)

    def hal_write_data(self, data):
        """Write data to the LCD."""
        byte = (MASK_RS | (self.backlight << SHIFT_BACKLIGHT) | (((data >> 4) & 0x0f) << SHIFT_DATA))
        self.i2c.writeto(self.i2c_addr, bytearray([byte | MASK_E]))
        self.i2c.writeto(self.i2c_addr, bytearray([byte]))
        byte = (MASK_RS | (self.backlight << SHIFT_BACKLIGHT) | ((data & 0x0f) << SHIFT_DATA))
        self.i2c.writeto(self.i2c_addr, bytearray([byte | MASK_E]))
        self.i2c.writeto(self.i2c_addr, bytearray([byte]))

class LCD1602(I2cLcd):
    def __init__(self):
        self.DEFAULT_I2C_ADDR = 0x27
        self.i2c = I2C(0,sda=Pin(20),scl=Pin(21),freq=400000)
        
    def lcd(self):
        self.lcd = I2cLcd(self.i2c, self.DEFAULT_I2C_ADDR, 2, 16)