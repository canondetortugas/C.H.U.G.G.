import threading

import numpy as np
import rospy
import tf

from chugg_physics.ChuggSimulator import ChuggSimulator

class ROSChuggSimulator(ChuggSimulator):
    def __init__(self):
        I = rospy.get_param('~I', ChuggSimulator.default_I)
        wheels = rospy.get_param('~wheels', ChuggSimulator.default_wheels)
        ChuggSimulator.__init__(self, I=I, wheels=wheels)
        
        self.loop_rate_hz = float(rospy.get_param('~loop_rate', 60))
        self.loop_rate = rospy.Rate(self.loop_rate_hz)

        self.last_update_time = None
        self.next_wheel_acc = None
        self.next_ext_torque = None

        # Ros resources
        self.tb = tf.TransformBroadcaster()

        # Spin up thread
        self.is_shutdown = False
        rospy.on_shutdown(self.cleanup)
        self.thread = threading.Thread(target=self.simulatorThread)
        self.thread.start()
    
    def setWheelAcc(self, acc):
        self.next_wheel_acc = acc

    def setExternalTorque(self, torque):
        self.next_ext_torque = torque

    def simulatorThread(self):
        while True:
            if self.last_update_time is None:
                dt = 1.0/self.loop_rate_hz
                self.last_update_time = rospy.Time.now()
            else:
                now = rospy.Time.now()
                dt = (now - self.last_update_time).to_sec()
                self.last_update_time = now


            if self.next_wheel_acc is None:
                acc = np.zeros(len(self.wheels))
            else:
                acc = self.next_wheel_acc
                self.next_wheel_acc = None

            if self.next_ext_torque is None:
                torque = np.zeros(3)
            else:
                torque = self.next_ext_torque
                self.next_ext_torque = None

            self.step(acc, ext_torque=torque, dt=dt)

            self.tb.sendTransform((0.0,0.0,0.0), self.ori, self.last_update_time, 
                                  "chugg/ori/sim", "/world")
            
            if self.is_shutdown:
                break
            self.loop_rate.sleep()

    def cleanup(self):
        self.is_shutdown = True
        self.thread.join()
