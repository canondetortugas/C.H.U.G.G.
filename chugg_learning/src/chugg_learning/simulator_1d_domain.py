import numpy as np

import rospy

import tf.transformations as tr

from chugg_learning.chugg_1d_domain import ChuggDomain1DBase, quat_distance
from chugg_physics.ChuggSimulator import ChuggSimulator, axisangle_to_quat
from chugg_physics.ROSChuggSimulator import ROSChuggSimulator

# Our quaternion convention: (w, x, y, z)
# Transformations.py convention: (w, x, y, z)
# Simulator convention: (x, y, z, w)

class OfflineChuggSimulator1DDomain(ChuggDomain1DBase):
    
    # Extra args are passed to physics simulator
    def __init__(self, sim_type=ChuggSimulator, **kwargs):
        self.sim_args = kwargs
        self.sim_type = sim_type
        self.sim = None
        super(OfflineChuggSimulator1DDomain, self).__init__()
        
    def s0(self):
        if self.sim is None:
            self.sim = self.sim_type(**self.sim_args)

        if self.random_start:
            self.state = self._randomState()
        else:
            self.state = self.default_state
            
        self.traj = [self.state]

        # Used to compute penalty for ending early
        self.current_step = 0

        # Set the physics simulator state
        (yaw, vel, wheel_vel) = self._state()
        # ori = to_sim_convention(ori)
        
        self.sim.setState(ori=axisangle_to_quat(np.array((0.0, 0.0, 1.0)), yaw), 
                          vel=np.array((0.0, 0.0, vel)), 
                          wheel_vel=np.array((0.0, 0.0, wheel_vel)))
                          
        # Implicitly converted to tuple
        return self.state.copy(), self.isTerminal(), self.possibleActions()


    def step(self, action_idx):
        action = self.actions[action_idx]
        acc = np.array((0.0, 0.0, action))

        # print self.sim.ori, self.sim.vel, self.sim.wheel_vel
        # Step the physics simulator
        self.sim.step(acc, dt=self.dt)
        self._updateStateFromSimulator()

        new_state = self.state.copy()
        self.traj.append(new_state)
        
        reward = self._getReward(action)
        
        return (reward, new_state, self.isTerminal(), self.possibleActions() )

    def _updateStateFromSimulator(self):
        ori = self.sim.ori
        (roll, pitch, yaw) = tr.euler_from_quaternion(ori, self.euler_convention)
        vel = self.sim.vel[2]
        wheel_vel = self.sim.wheel_vel[2]
        state = np.array((yaw, vel, wheel_vel))
        self.state = state

class OnlineChuggSimulator1DDomain(OfflineChuggSimulator1DDomain):

    speedup = 10

    def __init__(self):
        super(OnlineChuggSimulator1DDomain, self).__init__(sim_type=ROSChuggSimulator, spin_thread=False)

    def showDomain(self, a):
        self.sim.publishState()
        # print "Ori: ", self.sim.ori, " vel: ", self.sim.vel, " wheel_vel: ", self.sim.wheel_vel
        rospy.sleep(self.dt/self.speedup)
