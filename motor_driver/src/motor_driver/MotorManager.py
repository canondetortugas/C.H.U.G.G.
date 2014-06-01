import rospy
import wiringpi as wp
import smbus

from uscauv_common import DynamicReconfigureServer

from motor_driver.Motor import Motor
from motor_driver.cfg import MotorManagerConfig

import motor_driver.constants as mdc

class MotorManager:

    def __init__(self, names, ns='~motors'):
        self.ns = ns
        wp.wiringPiSetupSys()
        self.gpio = wp
        i2c_id = MotorManager.getPiI2CBusNumber()
        rospy.loginfo('Opening i2c bus [ {} ]'.format(i2c_id))
        self.bus = smbus.SMBus( i2c_id )
        
        self.motors = {name: Motor(self.bus, self.gpio, ns + '/' + name) for name in names}

        self.rc = DynamicReconfigureServer(MotorManagerConfig, self.reconfigureCallback, ns)

    def reconfigureCallback(self, config, levels):

        new_mode = mdc.MOTOR_MODES[config.mode]

        # Update motor modes
        for motor in self.motors.itervalues():
            motor.mode = new_mode
        self.gpio.digitalWrite(config.dig1_pin, new_mode.pin1)
        self.gpio.digitalWrite(config.dig2_pin, new_mode.pin2)
            
        self.gpio.digitalWrite(config.enable_pin, not config.hard_disable)
            
        self.config = config
        return config

    def setMotorVelocity(self, name, vel):
        self.motors[name].setVelocity(vel)

    @staticmethod
    def getPiRevision():
        "Gets the version number of the Raspberry Pi board"
        # Courtesy quick2wire-python-api
        # https://github.com/quick2wire/quick2wire-python-api
        try:
            with open('/proc/cpuinfo','r') as f:
                for line in f:
                    if line.startswith('Revision'):
                        return 1 if line.rstrip()[-1] in ['1','2'] else 2
        except:
            return 0

    @staticmethod
    def getPiI2CBusNumber():
        # Gets the I2C bus number /dev/i2c#
        return 1 if MotorManager.getPiRevision() > 1 else 0
