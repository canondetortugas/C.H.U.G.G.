from dynamic_reconfigure.parameter_generator_catkin import int_t
from dynamic_reconfigure.parameter_generator_catkin import str_t


# All GPIO pins except those used for I2C
GPIO_PINS = (7, 8, 9, 10, 11, 14, 15, 17, 18, 22, 23, 24, 25, 27)

# TODO: Change these ranges to correct values
MOTOR_RANGES_RPM = {'low': (62, 100), 'medium': (62, 1000), 'high': (62, 10000)}

def get_gpio_pin_enum(gen):
    return gen.enum( [gen.const('GPIO{}'.format(p), int_t, p, "A GPIO pin") for p in GPIO_PINS],
                     "GPIO pins" )

def get_motor_range_enum(gen):
    return gen.enum( [gen.const(rng, str_t, rng, 'Motor range') for rng in MOTOR_RANGES_RPM.iterkeys()],
                     "Motor ranges" )



