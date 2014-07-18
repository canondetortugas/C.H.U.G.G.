import threading

import numpy as np

import rospy

import tf.transformations as tr

from geometry_msgs import Vector3
from std_srvs import Empty

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

        self.sensors = self.sensors = ChuggSensorSubscriber('/chugg/ori/final', self.sensorCallback)
        self.vel_pub = rospy.Publisher('motor_driver/target_vel', Vector3)
        self.start_service = rospy.Service('start_learning', Empty, self.startEpisodeCallback)
        self.stop_service = rospy.Service('stop_learning', Empty, self.stopEpisodeCallback)
        
        self.threading_ready = False

        super(ChuggHardware1DDomain, self).__init__()

    def startThreading(self):
        if self.threading_ready:
            return
        
        self.start_cv = threading.Condition()
        self.sensor_cv = threading.Condition()

        self.waiting_for_sensor = False

    def sensorCallback(self, ori, vel, wheel_vel, last_update_time):
        if not self.waiting_for_sensor:
            return

        (roll, pitch, yaw) = tr.euler_from_quaternion(ori, self.euler_convention)
        vz = vel[2]
        wheelz = wheel_vel[2]
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
        self._waitForStateUpdate()

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
        interrupt = _userInterrupted()
        return term or interrupt

    def _setAcc(self, acc):
        new_wheel_vel = self.wheel_vel + acc*self.dt
        default = self.default_wheel_vel

        msg = Vector3()
        msg.x = default
        msg.y = default
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
        self.wheel_vel = wv
        msg = Vector3()
        msg.x = wv
        msg.y = wv
        msg.z = wv
        self.vel_pub.publish(msg)
