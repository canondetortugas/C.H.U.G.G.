import numpy as np

import rospy

import tf.transformations as tr

from geometry_msgs import Vector3
from std_srvs import Empty

from chugg_learning.chugg_1d_domain import ChuggDomain1DBase, quat_distance
from chugg_physics.ChuggSimulator import ChuggSimulator
from chugg_physics.ROSChuggSimulator import ROSChuggSimulator

from chugg_learning.utilities import ChuggSensorSubscriber

# Our quaternion convention: (w, x, y, z)
# Transformations.py convention: (w, x, y, z)
# Simulator convention: (x, y, z, w)
class ChuggHardwareDomain(ChuggDomain1DBase):

    default_wheel_vel = np.array((120.0, 120.0, 120.0))
    
    # Extra args are passed to physics simulator
    def __init__(self):

        self.random_start = None
        self.interrupt = False

        self.sensors = self.sensors = ChuggSensorSubscriber('/chugg/ori/final', self.sensorCallback)
        self.vel_pub = rospy.Publisher('motor_driver/target_vel', Vector3)
        
        super(ChuggHardwareDomain, self).__init__()

    def sensorCallback(self, ori, vel, wheel_vel, last_update_time):
        (roll, pitch, yaw) = tr.euler_from_quaternion(ori, self.euler_convention)
        vz = vel[2]
        wheelz = wheel_vel[2]
        self.state = np.array((yaw, vz, wheelz))

        # TODO: Signal waiting thread

    def s0(self):
        # Used to compute penalty for ending early
        self.current_step = 0
        self.interrupt = False

        # Wait for the user to press a key
        self._initializePlatform()
        self._waitForEpisodeStart()
        self._waitForStateUpdate()
            
        self.traj = [self.state]
        
        # Implicitly converted to tuple
        return self.state.copy(), self.isTerminal(), self.possibleActions()

    def step(self, action_idx):
        action = self.actions[action_idx]
        acc = action

        # Send motor command
        self._performAction(acc)
        # Wait for the state to update
        self._waitForStateUpdate()

        new_state = self.state.copy()
        self.traj.append(new_state)
        
        reward = self._getReward(action)
        
        return (reward, new_state, self.isTerminal(), self.possibleActions() )

    def _waitForStateUpdate(self):
        # TODO: this
        pass

    def _waitForEpisodeStart(self):
        rospy.loginfo('Waiting for user to start episode')
        # TODO: fill out
        pass
        
    def isTerminal(self):
        term = super(ChuggHardwareDomain, self).isTerminal()
        interrupt = _userInterrupted()
        # TODO: Check if the user interrupted the run
        return term or interrupt

    def _userInterrupted(self):
        return self.interrupt

    def _performAction(self, action):
        # TODO
        pass

    def startEpisodeCallback(self, request):
        # TODO: fill out
        return []

    def stopEpisodeCallback(self, request):
        rospy.loginfo('Episode interrupted')
        self.interrupt = True
        return []
    
    # Initialize hardware platform state
    def _initializePlatform(self):
        wv = self.default_wheel_vel
        msg = Vector3()
        msg.x = wv[0]
        msg.y = wv[1]
        msg.z = wv[2]
        self.vel_pub.publish(msg)
