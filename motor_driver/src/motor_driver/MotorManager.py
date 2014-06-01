import rospy
import wiringpi as wp

from uscauv_common import DynamicReconfigureServer

from motor_driver.Motor import Motor
from motor_driver.cfg import MotorManagerConfig

class MotorManager:

    def __init__(self, names, ns='~motors'):
        self.ns = ns
        wp.wiringPiSetupSys()
        self.gpio = wp
        self.bus = #TODO
        
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

        

