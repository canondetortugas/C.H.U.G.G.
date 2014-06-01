import rospy
import wiringpi as wp
import smbus

from uscauv_common import DynamicReconfigureServer

from motor_driver.Motor import Motor
from motor_driver.cfg import MotorManagerConfig

class MotorManager:

    def __init__(self, names, ns='~motors'):
        self.ns = ns
        wp.wiringPiSetupSys()
        self.gpio = wp
        self.bus = smbus.SMBus( MotorManager.getPiI2CBusNumber() )
        
        self.motors = {name: Motor(bus, gpio, ns + '/' + name) for name in names}

        self.rc = DynamicReconfigureServer(MotorManagerConfig, reconfigureCallback, ns)

    def reconfigureCallback(config, levels):

        new_mode = mdc.MOTOR_MODES[config.mode]

        # Update motor modes
        for motor in self.motors.itervals():
            motor.mode = new_mode
        self.gpio.digitalWrite(config.dig1_pin, new_mode.pin1)
        self.gpio.digitalWrite(config.dig2_pin, new_mode.pin2)
            
        self.gpio.digitalWrite(config.enable_pin, not config.hard_disable)
            
        self.config = config
        return config

    def setMotorVelocity(name, vel):
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
        return 1 if Adafruit_I2C.getPiRevision() > 1 else 0