import threading

import numpy as np
import rospy
import tf

from chugg_physics.ChuggSimulator import ChuggSimulator
from chugg_physics.ChuggSimulator import quat_mult, axisangle_to_quat

class ROSChuggSimulator(ChuggSimulator):
    def __init__(self, spin_thread=True):
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
        self.spin_thread = spin_thread
        if self.spin_thread:
            self.is_shutdown = False
            rospy.on_shutdown(self.cleanup)
            self.thread = threading.Thread(target=self.simulatorThread)
            self.thread.start()
    
    def setWheelAcc(self, acc):
        if len(acc) != self.n:
            rospy.logwarn("Wrong number of reaction wheels. Ignoring...")
            return
            
        self.next_wheel_acc = acc

    def setWheelTorque(self, tq):
        print "DISABLED!"
        assert False
        if len(tq) != self.n:
            rospy.logwarn("Wrong number of reaction wheels. Ignoring...")
            return
        
        # self.next_wheel_acc = np.matrix(self.J).getI().dot(tq).A1

    def setExternalTorque(self, torque):
        self.next_ext_torque = torque

    def stepWheelVelocity(self, target_wheel_vel, dt):
        target_wheel_vel = np.array(target_wheel_vel)
        current_vel = self.wheel_vel
        wheel_acc = target_wheel_vel - current_vel
        ChuggSimulator.step(self, wheel_acc, ext_torque=None, dt=dt)

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
                torque = None
            else:
                torque = self.next_ext_torque
                self.next_ext_torque = None

            # Simulate and publish
            ChuggSimulator.step(self, acc, ext_torque=torque, dt=dt)
            self.publishState()
            
            if self.is_shutdown:
                break
            self.loop_rate.sleep()

    def publishState(self):
        self.tb.sendTransform((0.0,0.0,0.0), self.ori, self.last_update_time, 
                                  "chugg/ori/sim", "/world")

        # Publish wheel transforms
        for (wheel, angle) in zip(self.wheels, self.wheel_pos):
            # Rotation about the wheel's axis in a coordinate frame fixed with its x axis aligned with the wheel axis
            wpc = axisangle_to_quat(np.array((1.0, 0.0, 0.0)), angle)
            ori = quat_mult(wheel.axis_transform, wpc)
            self.tb.sendTransform(wheel.t, ori, self.last_update_time,
                                  'chugg/wheels/{}'.format(wheel.name), 'chugg/ori/sim')

    def cleanup(self):
        self.is_shutdown = True
        self.thread.join()
