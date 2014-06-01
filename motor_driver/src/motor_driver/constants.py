from dynamic_reconfigure.parameter_generator_catkin import int_t
from dynamic_reconfigure.parameter_generator_catkin import str_t

from enum import Enum

CW = 0
CCW = 1

POSITIVE_MOTOR_DIR = CCW

class DigMode:
    # Range is in RPM units
    def __init__(self, rng, pin1, pin2):
        self.rng = rng
        self.pin1 = pin1
        self.pin2 = pin2

VOLTAGE_MIN = 0.0
VOLTAGE_MAX = 5.0

# All GPIO pins except those used for I2C
GPIO_PINS = (7, 8, 9, 10, 11, 14, 15, 17, 18, 22, 23, 24, 25, 27)

# TODO: Change these ranges to correct values
MOTOR_MODES = {'low': DigMode((62, 100), 0, 1), 
               'medium': DigMode((62, 1000), 1, 0), 
               'high': DigMode((62, 10000), 1, 1)}

def get_gpio_pin_enum(gen):
    return gen.enum( [gen.const('GPIO{}'.format(p), int_t, p, "A GPIO pin") for p in GPIO_PINS],
                     "GPIO pins" )

def get_motor_mode_enum(gen):
    return gen.enum( [gen.const('{}'.format(dm.rng), str_t, name, 'Motor range') for (name, dm) in MOTOR_MODES.iteritems()],
                     "Motor ranges" )



