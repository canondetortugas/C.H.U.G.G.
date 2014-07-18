import numpy as np
from math import pi
import rospy
import tf
from geometry_msgs.msg import Vector3Stamped

import subprocess
from os import path

def get_log_dir():
    pkg_dir = subprocess.check_output(['rospack', 'find', 'chugg_learning'])
    pkg_dir = pkg_dir[:-1]      # remove newline
    
    return path.join(pkg_dir, 'logs')

def get_log_path():
   
    return path.join(get_log_dir(), '{domain}/{agent}/{representation}/')

def sdoc(a, b):
    e1 = (a - b) % (2 * pi)
    e2 = (b - a) % (2 * pi)
    
    if e2 > e1:
        return -e1
    else:
        return e2

# (x, y, z, w) quaternion convention
class ChuggSensorSubscriber(object):

    def __init__(self, ori_frame, callback):

        self.ori = np.array((0.0, 0.0, 0.0, 1.0))
        self.vel = np.zeros(3)
        self.wheel_vel = np.zeros(3)

        self.ori_frame = ori_frame

        self.tl = tf.TransformListener()
        self.vel_sub = rospy.Subscriber('imu_driver/angular_rate', Vector3Stamped, self.velCallback)
        self.wheel_vel_sub = rospy.Subscriber('~wheel_vel', Vector3Stamped, self.wheelVelCallback)
        
        self.callback = callback
        self.last_update_time = None

    def updateOrientation(self):
        parent_frame = '/world'
        try:
            (pos, ori) = self.tl.lookupTransform(parent_frame, self.ori_frame, rospy.Time(0) )
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr('ChuggSensorSubscriber failed to lookup transform [ {} ] to [ {} ].'.format(parent_frame, self.ori_frame))
            return
        self.ori = ori

    def velCallback(self, msg):
        # TODO: Transform vector into the cube's body frame
        vel = msg.vector
        self.last_update_time = msg.header.stamp
        self.vel = np.array((vel.x, vel.y, vel.z))

        self.updateOrientation()
        self.update()

    def wheelVelCallback(self, msg):
        wheel_vel = msg.vector
        self.last_update_time = msg.header.stamp
        self.wheel_vel = wheel_vel

        self.updateOrientation()
        self.update()

    def update(self):
        try:
            self.callback(self.ori, self.vel, self.wheel_vel, self.last_update_time)
        except Exception as e:
            rospy.logerr('External callback failed [ {} ].'.format(e))
            raise e
