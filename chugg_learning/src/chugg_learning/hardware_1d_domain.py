import threading

import numpy as np

import rospy

import tf.transformations as tr

from geometry_msgs.msg import Vector3
from std_srvs.srv import Empty

from chugg_learning.chugg_1d_domain import ChuggDomain1DBase, quat_distance
from chugg_physics.ChuggSimulator import ChuggSimulator
from chugg_physics.ROSChuggSimulator import ROSChuggSimulator

from chugg_learning.utilities import ChuggSensorSubscriber

# Quaternion convention: (x, y, z, w)
class ChuggHardware1DDomain(ChuggDomain1DBase):

    default_wheel_vel = np.array((120.0, 120.0, 120.0))
    
    # Extra args are passed to physics simulator
    def __init__(self):

        self.random_start = None
        self.interrupt = False
        self.wheel_vel = 0.0

        print "Setting threading ready false"
        self.threading_ready = False

        super(ChuggHardware1DDomain, self).__init__()

    def startThreading(self):
        print "Threading ready: ", self.threading_ready
        if self.threading_ready is True:
            return

        print "Setting threading ready true"
        self.threading_ready = True

        self.sensors = self.sensors = ChuggSensorSubscriber('/chugg/ori/final', self.sensorCallback)
        self.vel_pub = rospy.Publisher('motor_driver/target_vel', Vector3)
        self.start_service = rospy.Service('start_learning', Empty, self.startEpisodeCallback)
        self.stop_service = rospy.Service('stop_learning', Empty, self.stopEpisodeCallback)

        self.start_cv = threading.Condition()
        self.sensor_cv = threading.Condition()

        self.waiting_for_sensor = False

    def sensorCallback(self, ori, vel, wheel_vel, last_update_time):
        # print "Sensors: ", ori, vel
        if not self.waiting_for_sensor:
            return

        (roll, pitch, yaw) = tr.euler_from_quaternion(ori, self.euler_convention)
        vz = vel[2]
        # wheelz = wheel_vel[2]
        wheelz = self.wheel_vel

        # print yaw, vz, wheelz
        self.state = np.array((yaw, vz, wheelz))
        
        with self.sensor_cv:
            self.sensor_cv.notify_all()

    def s0(self):
        self.startThreading()

        # Used to compute penalty for ending early
        self.current_step = 0
        self.interrupt = False

        # Wait for the user to press a key
        self._initializePlatform()
        self._waitForEpisodeStart()
        rospy.loginfo('Waiting for first sensor measurement.')
        self._waitForStateUpdate()
        rospy.loginfo('Got sensor measurement. Proceeding...')

        state = self.state.copy()
        
        self.traj = [state]
        
        # Implicitly converted to tuple
        return state.copy(), self.isTerminal(), self.possibleActions()

    def step(self, action_idx):
        action = self.actions[action_idx]
        acc = action

        # Send motor command
        self._setAcc(acc)
        # Wait for the state to update
        self._waitForStateUpdate()

        new_state = self.state.copy()
        self.traj.append(new_state)
        
        reward = self._getReward(action)
        
        return (reward, new_state, self.isTerminal(), self.possibleActions() )

    def _waitForStateUpdate(self):
        self.waiting_for_sensor = True

        with self.sensor_cv:
            self.sensor_cv.wait()

        self.waiting_for_sensor = False

    def _waitForEpisodeStart(self):
        rospy.loginfo('Waiting for user to start episode...')

        with self.start_cv:
            self.start_cv.wait()

        rospy.loginfo("Starting episode.")
        
    def isTerminal(self):
        term = super(ChuggHardware1DDomain, self).isTerminal()
        interrupt = self._userInterrupted()
        return term or interrupt

    def _setAcc(self, acc):
        new_wheel_vel = self.wheel_vel + acc*self.dt
        default = self.default_wheel_vel

        msg = Vector3()
        msg.x = default[0]
        msg.y = default[1]
        msg.z = new_wheel_vel
        self.wheel_vel = new_wheel_vel
        
        self.vel_pub.publish(msg)

    def startEpisodeCallback(self, request):
        with self.start_cv:
            self.start_cv.notify_all()
        return []

    def stopEpisodeCallback(self, request):
        rospy.loginfo('Episode interrupted.')
        self.interrupt = True
        return []

    def _userInterrupted(self):
        return self.interrupt
    
    # Initialize hardware platform state
    def _initializePlatform(self):
        wv = self.default_wheel_vel
        self.wheel_vel = wv[2]
        msg = Vector3()
        msg.x = wv[0]
        msg.y = wv[1]
        msg.z = wv[2]
        self.vel_pub.publish(msg)
