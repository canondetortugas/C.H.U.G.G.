import numpy as np

# Wheels: {x: {axis: (x,y,z), J: 9}}

def axisangle_to_quat(axis, angle):
    return np.hstack(( axis*np.sin(angle/2), np.cos(angle/2)))

def quat_mult(p,q):
    ''' Perform quaternion product q*p'''
    return np.array([p[0]*q[1] + q[0]*p[1] + p[2]*q[3] - p[3]*q[2],
                     p[0]*q[2] + q[0]*p[2] + p[3]*q[1] - p[1]*q[3],
                     p[0]*q[3] + q[0]*p[3] + p[1]*q[2] - p[2]*q[1],
                     p[0]*q[0] - (p[1]*q[1] + p[2]*q[2] + p[3]*q[3])])

def normalize(q):
    return q / numpy.linalg.norm(q)

# Quaternion: (x, y, z, w)
# TODO: Check whether or not we are setting wheel vel correctly.
# I don't think we are, because under the current model the wheels don't move if we apply a torque to the cube,
# which doesn't really make sense
class ChuggSimulator:
    default_I = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    default_wheels = [{'axis': (-1, 0, 0), 'J': 0.5}, 
                      {'axis': (0, -1, 0), 'J': 0.5}, 
                      {'axis': (0, 0, -1), 'J': 0.5} ]

    def __init__(self, I=None, wheels=None):
        if I is None:
            I = ChuggSimulator.default_I
        if wheels is None:
            wheels = ChuggSimulator.default_wheels

        self.I = np.matrix(I)
        self.wheels = wheels
        self.n = len(self.wheels)

        self.J = np.diag( [x['J'] for x in self.wheels] )
        
        # Horizontal concatenation. G is Nx3
        self.G = np.concatenate( [ np.matrix(g['axis']).transpose() for g in self.wheels], axis=1)

        self.Jp = self.G.dot(self.J.dot(self.G.transpose() ) )

        # System state
        self.ori = np.array((0.0,0.0,0.0,1.0))
        self.vel = np.array((0.0,0.0,0.0))
        self.wheel_vel = np.zeros(len(wheels))
        self.wheel_pos = np.zeros(len(wheels))
        
    def step(self, wheel_acc, ext_torque = None, dt = 0.02):
        if len(wheel_acc) != self.n:
            raise ValueError('Wrong number of reaction wheels.')
        
        if ext_torque is None:
            ext_torque = np.array((0.0,0.0,0.0))

        w = self.vel
        O = self.wheel_vel
        Odot = np.array(wheel_acc)

        h = self.J.dot((O + self.G.transpose().dot(w)).A1)

        wdot = ((self.I + self.Jp).getI().dot(
                (-np.cross(self.vel, self.I.dot(self.vel)) - self.G.dot(self.J).dot(Odot)
                  - np.cross(self.vel, self.G.dot(h)) + ext_torque).A1)).A1

        # Integrate
        if np.linalg.norm(self.vel) != 0.0:
            theta = np.linalg.norm(self.vel)*dt
            axis = self.vel / np.linalg.norm(self.vel)
            deltaq = axisangle_to_quat(axis, theta)
        else:
            deltaq = np.array((0,0,0,1))
            
        self.ori = quat_mult(self.ori, deltaq)
            
        self.vel += wdot*dt
        
        self.wheel_pos += self.wheel_vel*dt
        self.wheel_pos %= (2*np.pi)
        
        self.wheel_vel += Odot*dt
        
    def setState(self, ori=(0,0,0,1), vel=(0,0,0), wheel_vel=None, wheel_pos = None):
        if wheel_vel == None:
            wheel_vel =  np.zeros(len(self.wheels))
        else:
            wheel_vel = np.array(wheel_vel)
        if wheel_pos == None:
            wheel_pos = np.zeros(len(self.wheels))
        else:
            wheel_pos = np.array([x % (2*np.pi) for x in wheel_pos])
        
        self.ori = np.array(ori)
        self.vel = np.array(vel)
