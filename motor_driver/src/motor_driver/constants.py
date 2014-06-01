from dynamic_reconfigure.parameter_generator_catkin import int_t
from dynamic_reconfigure.parameter_generator_catkin import str_t

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

DEFAULT_MOTOR_ADDRESS = '0x60'

# All GPIO pins except those used for I2C
GPIO_PINS = (7, 8, 9, 10, 11, 14, 15, 17, 18, 22, 23, 24, 25, 27)

MOTOR_MODES = {'low': DigMode((62, 625), 0, 1), 
               'medium': DigMode((62, 2500), 1, 0), 
               'high': DigMode((62, 10000), 1, 1)}

def get_gpio_pin_enum(gen):
    return gen.enum( [gen.const('GPIO{}'.format(p), int_t, p, "A GPIO pin") for p in GPIO_PINS],
                     "GPIO pins" )

def get_motor_mode_enum(gen):
    return gen.enum( [gen.const(name, str_t, name, 'Motor range') for (name, dm) in MOTOR_MODES.iteritems()],
                     "Motor ranges" )



