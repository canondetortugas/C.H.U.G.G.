import rospy

from uscauv_common import DynamicReconfigureServer
from motor_driver.cfg import MotorConfig

from math import pi

RPM_TO_RADS=2*pi/60.0

def rads_to_rpm(x):
    return x/RPM_TO_RADS
def rpm_to_rads(x):
    return x*RPM_TO_RADS

class Motor:

    def __init__(self, bus, gpio, ns='~'):
        self.ns = ns
        self.bus = bus
        self.gpio = gpio
        self.config = None

        # ROS stuff
        self.rc = DynamicReconfigureServer(MotorConfig, self.reconfigureCallback, ns)

    def reconfigureCallback(self, config, level):
        self.config = config
        return config

    # Velocity in radians / sec
    def setVelocity(self, vel):
        rpm = rads_to_rpm(vel)

        

        pass
    
    def __setVoltage(self, volts):
        pass

    
