import numpy as np
from collections import OrderedDict

# Wheels: {x: {axis: (x,y,z), J: 9}}

def axisangle_to_quat(axis, angle):
    return np.hstack( axis*np.sin(angle/2), np.cos(angle/2))

def quat_mult(p,q):
    ''' Perform quaternion product q*p'''
    return array([p[0]*q[0] - (p[1]*q[1] + p[2]*q[2] + p[3]*q[3]),
                  p[0]*q[1] + q[0]*p[1] + p[2]*q[3] - p[3]*q[2],
                  p[0]*q[2] + q[0]*p[2] + p[3]*q[1] - p[1]*q[3],
                  p[0]*q[3] + q[0]*p[3] + p[1]*q[2] - p[2]*q[1]])

class ChuggSimulator:

    def __init__(self, I, wheels):

        self.I = np.matrix(I)
        self.wheels = OrderedDict(wheels) # Guarantees that we can iterate over wheels in a consistent order
        self.n = len(self.wheels)

        self.J = np.diag( [x['J'] for x in self.wheels.itervalues()] )

        self.G = np.concatenate( [ np.matrix(g['axis']).transpose() for g in self.wheels.itervalues() ], 0)

        self.Jp = self.G.dot(self.J.dot(self.G.transpose() ) )

        # System state
        self.ori = np.array((0,0,0,1))
        self.vel = np.array((0,0,0))
        self.wheel_vel = np.zeros((1, len(motors)))


    def step(dt = 0.02, wheel_acc, ext_torque = None):
        if len(wheel_acc) != self.n:
            raise ValueError('Wrong number of reaction wheels.')
        
        if ext_torque is None:
            ext_torque = np.array((0,0,0))

        w = self.vel
        O = self.wheel_vel
        Odot = np.array(wheel_acc)

        h = self.J.dot(self.wheel_vel + self.G.transpose().dot(self.wheel))
        
        wdot = ((self.I + self.Jp).getI()*
                (-np.cross(self.vel, self.I.dot(self.vel)) - self.G.dot(self.J).dot(Odot)
                  - np.cross(self.vel, self.G.dot(h)) + ext_torque))

        # Integrate
        theta = np.linalg.norm(self.vel)*dt
        axis = self.vel / np.linalg.norm(self.vel)
        deltaq = axisangle_to_quat(axis, theta)
        self.ori = quat_mult(ori, deltaq)

        self.vel += wdot*dt
        self.wheel_vel += wheel_acc*dt
        
    def setState(ori=(0,0,0,1), vel=(0,0,0), wheel_vel=None):
        if wheel_vel == None:
            wheel_vel =  np.zeros((1, len(motors)))
        
