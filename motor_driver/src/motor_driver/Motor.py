import rospy

from uscauv_common import DynamicReconfigureServer

from motor_driver.cfg import MotorConfig
import motor_driver.constants as mdc
from motor_driver.msg import Range

import threading

from math import pi

RPM_TO_RADS=mdc.RPM_TO_RADS

def rads_to_rpm(x):
    return x/RPM_TO_RADS
def rpm_to_rads(x):
    return x*RPM_TO_RADS

def rpm_to_volts(x, mn, mx):
    return float(x - mn)/float(mx - mn)*4.9 + 0.1

def volts_to_rpm(v, mn, mx):
    return clamp((v - 0.1)*(mx - mn)/4.9 + mn, mn, mx)

def clamp(val, min_, max_):
    if val < min_:
        out = min_
    elif val > max_:
        out = max_
    else:
        out = val
    return out

class Motor:

    def __init__(self, bus, gpio, ns='~'):
        self.ns = ns
        self.bus = bus
        self.gpio = gpio
        self.config = None
        self.dirn = None
        self.address = None
        self.thread = None

        self.pub_rate = rospy.get_param(ns + '/' + 'pub_rate', 10.0)
        self.pub = rospy.Publisher(ns + '/' + 'range', Range)

        self.mode = mdc.MOTOR_MODES.values()[0]

        # ROS
        self.rc = DynamicReconfigureServer(MotorConfig, self.reconfigureCallback, ns)

    def reconfigureCallback(self, config, level):

        try:
            self.address = int(config.address, 16)
            if self.address >> 7 != 0:
                raise ValueError("I2C address must fit in 7 bits")
        except ValueError as e:
            rospy.logerr('Bad address given: [ {} ]'.format(e))
            self.address = int(mdc.DEFAULT_MOTOR_ADDRESS, 16)
            rospy.logerr('Using default address: 0x{:x}'.format(self.address))
            
        self.config = config

        if not self.thread:
            self.thread = threading.Thread(group=None, target=self.rangeThread)
            self.lock = threading.Lock()
            self.is_shutdown = False
            rospy.on_shutdown(self.cleanup)
            self.thread.start()
            rospy.loginfo('Initializing motor [ {} ] to [ {} ] rad/s'.format(self.ns, mdc.MOTOR_MIN_RADS_ABS))
            self.setVelocity(mdc.MOTOR_MIN_RADS_ABS)

        return config
    
    def cleanup(self):
        self.is_shutdown = True
        self.thread.join()

    def setMode(self, mode):
        with self.lock:
            self.mode = mode

    def publishRange(self):
        msg = Range()
        with self.lock:
            msg.min = rpm_to_rads(self.mode.rng[0])
            msg.max = rpm_to_rads(self.mode.rng[1])
        self.pub.publish(msg)

    def rangeThread(self):
        while not rospy.is_shutdown() and not self.is_shutdown:
            self.publishRange()
            rospy.sleep(1.0/self.pub_rate)

    # Velocity in radians / sec
    def setVelocity(self, vel):
        if self.config.invert:
            vel = -vel
            
        if self.config.soft_disable:
            # Minimum RPM for the current mode
            rpm = self.mode.rng[0]
            # Use positive motor direction as the default direction
            dirn = mdc.POSITIVE_MOTOR_DIR
        else:
            if mdc.POSITIVE_MOTOR_DIR == mdc.CCW:
                dirn = 1 if vel >= 0 else 0
            elif mdc.POSITIVE_MOTOR_DIR == mdc.CW:
                dirn = 0 if vel >= 0 else 1
            rpm = rads_to_rpm(vel if vel >= 0 else -vel)

        if dirn != self.dirn:
            self.gpio.digitalWrite(self.config.dir_pin, dirn)
            self.dirn = dirn
            
        v = rpm_to_volts(rpm, self.mode.rng[0], self.mode.rng[1])
        self.__setVoltage(v + self.config.offset_voltage)

        
    def __setVoltage(self, volts):
        # print "Volts in ", volts

        volts = clamp(volts, mdc.VOLTAGE_MIN, mdc.VOLTAGE_MAX)

        # rospy.loginfo('Motor [ {} ] setting output voltage to [ {} ]'.format(self.ns, volts))

        dn = int(volts/mdc.VOLTAGE_MAX*4095)
        # dn = 2048             
        
        if not self.config.dummy_i2c:
            # Write using the MCP4725 "Write DAC register" mode (not fast write mode)
            try:
                self.bus.write_i2c_block_data(self.address, 0x40, 
                                              [(dn >> 4) & 0xff,
                                               (dn << 4) & 0xff])
            except IOError as e:
                rospy.logerr("GPIO error: [ {} ]".format(e))
        
