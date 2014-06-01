import rospy

from uscauv_common import DynamicReconfigureServer
from motor_driver.cfg import MotorConfig

import motor_driver.constants as mdc

from math import pi

RPM_TO_RADS=2*pi/60.0

def rads_to_rpm(x):
    return x/RPM_TO_RADS
def rpm_to_rads(x):
    return x*RPM_TO_RADS

def rpm_to_volts(x, mn, mx):
    return float(x - mn)/float(mx - mn)*4.9 + 0.1

class Motor:

    def __init__(self, bus, gpio, ns='~'):
        self.ns = ns
        self.bus = bus
        self.gpio = gpio
        self.config = None
        self.dir = None
        self.address = None

        self.mode = mdc.MOTOR_MODES.keys()[0]

        # ROS
        self.rc = DynamicReconfigureServer(MotorConfig, self.reconfigureCallback, ns)

    def reconfigureCallback(self, config, level):

        try:
            self.address = int(config.address, 16)
            if self.address >> 7 != 0:
                raise ValueError("I2C address must fit in 7 bits")
        except ValueError as e:
            rospy.logerror('Bad address given: [ {} ]'.format(e))
            self.address = int(mdc.DEFAULT_MOTOR_ADDRESS, 16)
            rospy.logerror('Using default address: 0x{:x}'.format(self.address))
            
        self.config = config
        return config

    # Velocity in radians / sec
    def setVelocity(self, vel):
        if self.config.invert:
            vel = -vel
            
        if self.config.soft_disable:
            # Minimum RPM for the current mode
            rpm = self.mode.rng[0]
            # Use positive motor direction as the default direction
            dir = mdc.POSITIVE_MOTOR_DIR
        else:
            rpm = rads_to_rpm(vel)
            if mdc.POSITIVE_MOTOR_DIR == mdc.CCW:
                dir = 1 if vel >= 0 else 0
            elif mdc.POSITIVE_MOTOR_DIR == mdc.CW:
                dir = 0 if vel >= 0 else 1

        if dir != self.dir:
            gpio.digitalWrite(self.config.dir_pin, dir)
            self.dir = dir
            
        v = rpm_to_volts(rpm, mode.rng[0], mode.rng[1])
        self.__setVoltage(v + self.config.offset_voltage)

        
    def __setVoltage(self, volts):
        
        if volts < mdc.VOLTAGE_MIN:
            volts = mdc.VOLTAGE_MIN
        if volts > mdc.VOLTAGE_MAX:
            volts = mdc.VOLTAGE_MAX

        dn = volts/mdc.VOLTAGE_MAX*4095
        
        # Write using the MCP4725 "Write DAC register" mode (not fast write mode)
        self.bus.write_i2c_block_data(self.address, 0x40, 
                                      [(dn >> 4) & 0xff,
                                       (dn << 4) & 0xff])
