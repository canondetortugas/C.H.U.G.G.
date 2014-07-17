import threading

import numpy as np
import rospy
import tf

from chugg_physics.ChuggSimulator import ChuggSimulator
from chugg_physics.ChuggSimulator import quat_mult, axisangle_to_quat

from geometry_msgs.msg import Vector3Stamped

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
        self.rate_pub = rospy.Publisher('imu_driver/angular_rate', Vector3Stamped)
        
        # Spin up thread
        self.spin_thread = spin_thread
        if self.spin_thread:
            self.is_shutdown = False
            rospy.on_shutdown(self.cleanup)
            self.thread = threading.Thread(target=self.simulatorThread)
            self.thread.start()
    
    def setWheelAcc(self, acc):
        '''Set next wheel acceleration that will be applied if a dedicated simulation thread is running'''
        if len(acc) != self.n:
            rospy.logwarn("Wrong number of reaction wheels. Ignoring...")
            return
            
        self.next_wheel_acc = acc

    # def setWheelTorque(self, tq):
    #     print "DISABLED!"
    #     assert False
    #     if len(tq) != self.n:
    #         rospy.logwarn("Wrong number of reaction wheels. Ignoring...")
    #         return
        
        # self.next_wheel_acc = np.matrix(self.J).getI().dot(tq).A1

    def setExternalTorque(self, torque):
        '''Set the next torque that will be applied if a dedicated simulation thread is running'''
        self.next_ext_torque = torque

    def stepWheelVelocity(self, target_wheel_vel, dt):
        '''Calculate the acceleration that would cause the wheel velocities to hit
        target_wheel_vel if the motors were not torque bounded, then step the physics
        simulation with this acceleration'''
        target_wheel_vel = np.array(target_wheel_vel)
        current_vel = self.wheel_vel
        wheel_acc = target_wheel_vel - current_vel
        self.step( wheel_acc, ext_torque=None, dt=dt)

    def spinWheelsToVel(self, vel):
        '''Block until the wheels have spun up to the velocities specified'''
        start = rospy.Time.now()
        tolerance = 1.0
        vel = np.array(vel)

        def complete():
            for (v, w) in zip(vel, self.wheel_vel):
                if abs(v -w) > tolerance:
                    return False
            return True

        idx = 0
        while True:
            if self.next_wheel_acc:
                continue
            
            max_acc = self.maxWheelAcc()
            acc = [min(m, abs(v - w))*np.sign(v - w) for (v, w, m) in zip(vel, self.wheel_vel, max_acc)]
            self.setWheelAcc(acc)

            if not (idx % 300):
                print "Current wheel velocity: ", self.wheel_vel

            if complete():
                print "Spun up in {} seconds.".format((rospy.Time.now() - start).to_sec() )
                break

    def simulatorThread(self):
        '''Continuously step the physics simulation and publish the state'''
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
            self.step( acc, ext_torque=torque, dt=dt)
            self.publishState()
            
            if self.is_shutdown:
                break
            self.loop_rate.sleep()

    def step(self, *args, **kwargs):
        ChuggSimulator.step(self, *args, **kwargs)
        self.last_update_time = rospy.Time.now()

    def publishState(self):
        '''Publish the orientation and wheel state.'''
        self.tb.sendTransform((0.0,0.0,0.0), self.ori, self.last_update_time, 
                                  "chugg/ori/final", "/world")

        rate_msg = Vector3Stamped()
        rate_msg.vector.x = self.vel[0]
        rate_msg.vector.y = self.vel[1]
        rate_msg.vector.z = self.vel[2]
        rate_msg.header.stamp = self.last_update_time
        rate_msg.header.frame_id = "chugg/cm"
        
        self.rate_pub.publish(rate_msg)

        # Publish wheel transforms
        for (wheel, angle) in zip(self.wheels, self.wheel_pos):
            # Rotation about the wheel's axis in a coordinate frame fixed with its x axis aligned with the wheel axis
            wpc = axisangle_to_quat(np.array((1.0, 0.0, 0.0)), angle)
            ori = quat_mult(wheel.axis_transform, wpc)
            self.tb.sendTransform(wheel.t, ori, self.last_update_time,
                                  'chugg/wheels/{}'.format(wheel.name), 'chugg/ori/final')

            

    def cleanup(self):
        '''Shut down the dedicated simulation thread'''
        self.is_shutdown = True
        self.thread.join()
