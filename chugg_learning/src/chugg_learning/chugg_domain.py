import numpy as np
import numpy.random as rnd
from numpy.linalg import norm
from math import pi

import tf.transformations as tr

from rlpy.Domains.Domain import Domain

import matplotlib.pyplot as plt

# TOOD: Wrong, transformations uses (x, y, z, w), switch to this 
# Our quaternion convention: (w, x, y, z)
# Transformations.py convention: (w, x, y, z)
# Simulator convention: (x, y, z, w)

class StateIndex:
    ROLL = 0
    PITCH = 1
    YAW = 2
    WX = 3
    WY = 4
    WZ = 5
    WHEELX = 6
    WHEELY = 7
    WHEELZ = 8

def to_range(x, rng):
    '''Map a variable in range [0,1] to the given range pair'''
    assert rng[1] > rng[0]      # Could also use >=, but that's a sorta odd use case
    return x*(rng[1] - rng[0]) + rng[0]

def in_range(x, rng):
    '''Return true if x lies in the closed set given by range, otherwise return false'''
    assert rng[1] > rng[0]
    return x <= rng[1] and x >= rng[0]    

def make_actions(points):
    actions = []
    for p1 in points:
        for p2 in points:
            for p3 in points:
                actions.append((p1, p2, p3))
    return np.array(actions)

def quat_distance(q1, q2):
    '''Return the angle between the two quaternions in radians, which turns out to be a distance metric over rotations'''
    quat_tolerance = 0.0001
    assert abs(norm(q1)- 1.0) < quat_tolerance and abs(norm(q2) - 1.0) < quat_tolerance
    return np.arccos(abs(q1.dot(q2)))

class ChuggDomainBase(Domain):

    # s - fixed frame, r - body frame
    # Ex: xyz - Rotation about x, followed by rotation about y, followed by rotation about z
    euler_convention = 'sxyz'

    euler_limits = np.array(((0, 2*pi), (-pi, pi), (0, 2*pi)))
    single_vel_limit = (-20.0, 20.0) # rad /s
    vel_limits = np.array((single_vel_limit, single_vel_limit, single_vel_limit))
    single_wheel_vel_limit = (-250.0, 250.0)
    wheel_vel_limits = np.array([single_wheel_vel_limit for idx in range(3)])

    # Actions
    action_limits = np.array((-50.0, 50.0))
    nactions_per_wheel = 5
    actions_single_axis = np.linspace(action_limits[0], action_limits[1], nactions_per_wheel)
    actions = make_actions(actions_single_axis)

    episode_length_seconds = 25        # Number of steps, not a physical time
    dt = 0.01
    episode_length = int(episode_length_seconds/dt)
    
    discount_factor = 0.9

    random_start = False

    # Clockwise rotation of 90 deg around x axis
    default_ori = np.array((np.sqrt(2.0)/2, np.sqrt(2.0)/2, 0.0, 0.0))
    default_vel = np.array((0.0, 0.0, 0.0))
    default_wheel_vel = np.array((0.0, 0.0, 0.0))
    default_state = np.hstack((default_ori, default_vel, default_wheel_vel))

    # Identity rotation
    goal_ori = np.array((1.0, 0.0, 0.0, 0.0))
    goal_tolerance = pi/6        # Angle between current rotation and target in radians

    visualization_bins = 20
    visualization_labels = 5

    def __init__(self):

        # Things that need to be defined for the rlpy domain class
        self.statespace_limits = np.hstack((self.euler_limits, self.vel_limits, self.wheel_vel_limits))
        self.continuous_dims = [StateIndex.ROLL, StateIndex.PITCH, StateIndex.YAW,
                                StateIndex.WX, StateIndex.WY, StateIndex.WZ,
                                StateIndex.WHEELX, StateIndex.WHEELY, StateIndex.WHEELZ]
        self.episodeCap = self.episode_length
        self.actions_num = len(self.actions)
        
        super(ChuggDomainBase, self).__init__()

    def _ori(self, s = None):
        '''Return current orientation as a quaternion'''
        if s is None:
            s = self.state
        
        rpy = s[StateIndex.ROLL:(StateIndex.YAW+1)]
        (roll, pitch, yaw) = (rpy[0], rpy[1], rpy[2])
        return np.array(tr.quaternion_from_euler(roll, pitch, yaw, self.euler_convention))
    def _vel(self, s = None):
        '''Return current angular velocity'''
        if s is None:
            s = self.state
        return s[StateIndex.WX:(StateIndex.WZ+1)]
    def _wheelVel(self, s = None):
        '''Return current wheel velocity'''
        if s is None:
            s = self.state
        return s[StateIndex.WHEELX:(StateIndex.WHEELZ+1)]

    def _state(self, s = None):
        '''Return state as a tuple (ori, vel, wheel_vel)'''
        if s is None:
            s = self.state
        return (self._ori(s), self._vel(s), self._wheelVel(s))

    # Abstract method for Domain class

    # Abstract method for Domain class
    # TODO: Update
    # def step(self, action_id):
    #     # TODO: Process wheel velocities so that they lie in the set: [min, -62] U [62, max]
        
    #     action = self.actions[action_id]
    #     # print action
        
    #     state = self.state.copy()
    #     state[0] += state[1]*self.dt
    #     state[1] += action*self.dt
    #     self.current_step += 1
    #     # print self.current_step
    #     # print state

    #     new_state = state
    #     # print new_state
    #     self.state = new_state
    #     self.traj.append(new_state)
    #     reward = self._getReward(action)
        
    #     return (reward, new_state, self.isTerminal(), self.possibleActions() )

    def isTerminal(self, s = None):
        '''True if state lies within limits, otherwise false'''
        if s is None:
            s = self.state
        vel = self._vel(s)
        wheel_vel = self._wheelVel(s)
        vel_exceeded = not all([in_range(v, r) for (v,r) in zip(vel, self.vel_limits)])
        wheel_vel_exceeded = not all([in_range(w, r) for (w,r) in zip(wheel_vel, self.wheel_vel_limits)])
        if vel_exceeded:
            print "Velocity limit exceeded: ", vel
        if wheel_vel_exceeded:
            print "Wheel velocity limit exceeded: ", wheel_vel
            
        return vel_exceeded or wheel_vel_exceeded

    def _randomState(self):
        # Not an unbiased sample of rotations - this may be a problem
        return np.array([to_range(self.random_state.rand(), limit) for limit in self.statespace_limits])

    
    def _getReward(self, action):
        if self.isTerminal():
            return -10.0*(self.episode_length - self.current_step)
        elif self._atGoal():
            return 1.0
        else:
            ori = self._ori()
            return -quat_distance(ori, self.goal_ori)**2

    def _atGoal(self):
        ori = self._ori()
        return quat_distance(ori, self.goal_ori) < self.goal_tolerance

    def showDomain(self, a):
        pass
        # traj = np.array(self.traj)

        # plt.figure(1)
        # plt.clf()
        
        # plt.scatter(traj[:,0], traj[:,1], c=range(len(traj)))
        # plt.xlim(self.pos_limits[0], self.pos_limits[1])
        # plt.ylim(self.vel_limits[0], self.vel_limits[1])
        # plt.draw()
        
    def showLearning(self, representation):
        pass
        # x = np.linspace(self.pos_limits[0], self.pos_limits[1], self.visualization_bins)
        # xdot = np.linspace(self.vel_limits[0], self.vel_limits[1], self.visualization_bins)

        # xticks = np.arange(self.visualization_labels)*float(self.visualization_bins)/(self.visualization_labels-1)
        # yticks = xticks

        # xlabels = ['{0:.1f}'.format(xx) for xx in np.linspace(self.pos_limits[0], self.pos_limits[1], self.visualization_labels)]
        # ylabels = ['{0:.1f}'.format(yy) for yy in np.linspace(self.vel_limits[0], self.vel_limits[1], self.visualization_labels)]

        # V = np.zeros((len(xdot), len(x)))
        # pi = np.zeros((len(xdot), len(x)))

        # for (idx, xx) in enumerate(x):
        #     for (idy, yy) in enumerate(xdot):
        #         s = np.array((xx, yy))
        #         term = self.isTerminal(s)
        #         Qs = representation.Qs(s, term) # Value for all possible actions in state s
        #         As = self.possibleActions()
        #         # If multiple optimal actions, pick one randomly
        #         a = np.random.choice(As[Qs.max() == Qs])
        #         pi[idy, idx] = a
        #         V[idy, idx] = max(Qs)

        # plt.figure(2)
        # plt.clf()
        # plt.imshow(V, origin='lower')
        # plt.title('Value Function')
        # plt.colorbar()
        # plt.xticks(xticks, xlabels)
        # plt.yticks(yticks, ylabels)
        # plt.figure(3)
        # plt.clf()
        # plt.imshow(pi, origin='lower')
        # plt.title('Policy')
        # plt.xticks(xticks, xlabels)
        # plt.yticks(yticks, ylabels)
        # plt.colorbar()
        # plt.draw()

