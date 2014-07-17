import numpy as np
import numpy.random as rnd
from numpy.linalg import norm
from math import pi

import tf.transformations as tr

from rlpy.Domains.Domain import Domain

import matplotlib.pyplot as plt

from chugg_learning.utilities import sdoc
from chugg_physics.ChuggSimulator import axisangle_to_quat

# TOOD: Wrong, transformations uses (x, y, z, w), switch to this 
# Our quaternion convention: (w, x, y, z)
# Transformations.py convention: (w, x, y, z)
# Simulator convention: (x, y, z, w)

class StateIndex:
    YAW = 0
    WZ = 1
    WHEELZ = 2

def to_range(x, rng):
    '''Map a variable in range [0,1] to the given range pair'''
    assert rng[1] > rng[0]      # Could also use >=, but that's a sorta odd use case
    return x*(rng[1] - rng[0]) + rng[0]

def in_range(x, rng):
    '''Return true if x lies in the closed set given by range, otherwise return false'''
    assert rng[1] > rng[0]
    return x <= rng[1] and x >= rng[0]    

# def make_actions(points):
#     actions = []
#     for p1 in points:
#         for p2 in points:
#             for p3 in points:
#                 actions.append((p1, p2, p3))
#     return np.array(actions)

def quat_distance(q1, q2):
    '''Return the angle between the two quaternions in radians, which turns out to be a distance metric over rotations'''
    quat_tolerance = 0.0001
    assert abs(norm(q1)- 1.0) < quat_tolerance and abs(norm(q2) - 1.0) < quat_tolerance
    return np.arccos(abs(q1.dot(q2)))

class ChuggDomain1DBase(Domain):

    # s - fixed frame, r - body frame
    # Ex: xyz - Rotation about x, followed by rotation about y, followed by rotation about z
    euler_convention = 'sxyz'

    euler_limit = (0, 2*pi)
    vel_limit = (-20.0, 20.0) # rad /s
    wheel_vel_limit = (-250.0, 250.0)

    # Actions
    action_limits = np.array((-50.0, 50.0))
    nactions_per_wheel = 5
    actions_single_axis = np.linspace(action_limits[0], action_limits[1], nactions_per_wheel)
    actions = actions_single_axis

    episode_length_seconds = 25        # Number of steps, not a physical time
    dt = 0.01
    episode_length = int(episode_length_seconds/dt)
    
    discount_factor = 0.9

    random_start = False

    # Clockwise rotation of 90 deg around z axis
    default_yaw = pi/2
    default_ori = axisangle_to_quat(np.array((0.0,0.0,1.0)), default_yaw)
    default_vel = 0.0
    default_wheel_vel = 20.0
    default_state = np.array((default_yaw, default_vel, default_wheel_vel))

    # Identity rotation
    goal_yaw = 0.0
    goal_ori = axisangle_to_quat(np.array((0.0, 0.0, 1.0)), goal_yaw)
    goal_tolerance = pi/6        # Angle between current rotation and target in radians

    visualization_bins = 20
    visualization_labels = 5

    def __init__(self):

        # Things that need to be defined for the rlpy domain class
        self.statespace_limits = np.array((self.euler_limit, 
                                           self.vel_limit,
                                           self.wheel_vel_limit))
        self.continuous_dims = [StateIndex.YAW,
                                StateIndex.WZ,
                                StateIndex.WHEELZ]
        self.episodeCap = self.episode_length
        self.actions_num = len(self.actions)
        
        super(ChuggDomain1DBase, self).__init__()

    def _ori(self, s = None):
        '''Return current orientation as a quaternion'''
        if s is None:
            s = self.state
        return s[StateIndex.YAW]
    def _vel(self, s = None):
        '''Return current angular velocity'''
        if s is None:
            s = self.state
        return s[StateIndex.WZ]
    def _wheelVel(self, s = None):
        '''Return current wheel velocity'''
        if s is None:
            s = self.state
        return s[StateIndex.WHEELZ]

    def _state(self, s = None):
        '''Return state as a tuple (ori, vel, wheel_vel)'''
        if s is None:
            s = self.state
        return (self._ori(s), self._vel(s), self._wheelVel(s))

    def isTerminal(self, s = None):
        '''True if state lies within limits, otherwise false'''
        if s is None:
            s = self.state
        vel = self._vel(s)
        wheel_vel = self._wheelVel(s)
        vel_exceeded = not in_range(vel, self.vel_limit)
        # wheel_vel_exceeded = not all([in_range(w, r) for (w,r) in zip(wheel_vel, self.wheel_vel_limits)])
        if vel_exceeded:
            print "Velocity limit exceeded: ", vel
        # if wheel_vel_exceeded:
        #     print "Wheel velocity limit exceeded: ", wheel_vel
            
        return vel_exceeded

    def _randomState(self):
        passx
        # Not an unbiased sample of rotations - this may be a problem
        # return np.array([to_range(self.random_state.rand(), limit) for limit in self.statespace_limits])
    
    def _getReward(self, action):
        if self.isTerminal():
            return -10.0*(self.episode_length - self.current_step)
        elif self._atGoal():
            return 1.0
        else:
            ori = self._ori()
            return -abs( sdoc(self._ori(), self.goal_yaw) )**2

    def _atGoal(self):
        ori = self._ori()
        return abs( sdoc(self._ori(), self.goal_yaw) ) < self.goal_tolerance

    def showDomain(self, a):
        pass
