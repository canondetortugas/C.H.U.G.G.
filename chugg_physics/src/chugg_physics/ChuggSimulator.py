import numpy as np

# Wheels: {x: {axis: (x,y,z), J: 9}}

def normalize(q):
    return q / np.linalg.norm(q)

def axisangle_to_quat(axis, angle):
    return np.hstack(( axis*np.sin(angle/2), np.cos(angle/2)))

def quat_mult(p,q):
    ''' Perform quaternion product q*p'''
    return np.array([p[3]*q[0] + q[3]*p[0] + p[1]*q[2] - p[2]*q[1],
                     p[3]*q[1] + q[3]*p[1] + p[2]*q[0] - p[0]*q[2],
                     p[3]*q[2] + q[3]*p[2] + p[0]*q[1] - p[1]*q[0],
                     p[3]*q[3] - (p[0]*q[0] + p[1]*q[1] + p[2]*q[2])])

def quat_between(v, w):
    '''Quaternion that takes vector v into vector w'''
    v = normalize(v)
    w = normalize(w)
    if all(v == w):
        return np.array((0.0, 0.0, 0.0, 1.0))
    elif all(v == -w):
        return np.array((0.0, 0.0, 0.0, -1.0))
    else:
        axis = np.cross(v, w)
        angle = np.arccos(np.dot(v, w))
        return axisangle_to_quat(axis, angle)

def quat_to_rotation_matrix(q):
    '''Transform a quaternion representing a clockwise rotation 
    around an axis by its angle into a rotation matrix representing
    the same transformation'''
    q0 = q[3]
    q1 = q[0]
    q2 = q[1]
    q3 = q[2]

    return np.array([[2*q0**2 - 1 + 2*q1**2, 2*q1*q2 + 2*q0*q3, 2*q1*q3 - 2*q0*q2],
                      [2*q1*q2-2*q0*q3, 2*q0**2 - 1 + 2*q2**2, 2*q2*q3 + 2*q0*q1],
                      [2*q1*q3 + 2*q0*q2, 2*q2*q3 - 2*q0*q1, 2*q0**2 - 1 + 2*q3**2]])

def translation_to_inertial_matrix(t):
    '''Compute the componenent of the inertial matrix of a body due to 
    its center of mass being offset by t from the frame in which 
    the inertial matrix is being calculated'''
    tx = t[0]
    ty = t[1]
    tz = t[2]
    
    txx = ty**2 + tz**2
    tyy = tx**2 + tz**2
    tzz = tx**2 + ty**2
    txy = tx*ty
    txz = tx*tz
    tyz = ty*tz

    return np.array([[txx, -txy, -txz],
                    [-txy, tyy, -tyz],
                    [-txy, -tyz, tzz]])

def matrix_inverse(A):
    '''Inverse of a 2D numpy ndarray'''
    return np.matrix(A).getI().A

class Wheel():
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)

# Quaternion: (x, y, z, w)
# TODO: Check whether or not we are setting wheel vel correctly.
# I don't think we are, because under the current model the wheels don't move if we apply a torque to the cube,
# which doesn't really make sense
class ChuggSimulator:
    default_I = [[.57, 0, 0], [0, .57, 0], [0, 0, .57]]
    default_J = [[0.007, 0, 0], [0, 0.0035, 0], [0, 0, 0.0035]]
    default_wheels = [{'axis': (1, 0, 0), 'J': default_J, 'mass': 0.2086525,
                       'pos': (-0.122, 0, 0), 'ori': (0.0, 0.0, 0.0, 1.0), 'name': 'x'},
                      {'axis': (0, 1, 0), 'J': default_J, 'mass': 0.2086525,
                       'pos': (0, -0.122, 0), 'ori': (0.0, 0.0, np.sqrt(2.0)/2.0, np.sqrt(2.0)/2.0), 'name': 'y'},
                      {'axis': (0, 0, -1), 'J': default_J, 'mass': 0.2086525,
                       'pos': (0, 0, .122), 'ori': (0.0, np.sqrt(2.0)/2.0, 0.0, np.sqrt(2.0)/2.0), 'name': 'z'} ]
    dtype=np.float64

    def __init__(self, I=None, wheels=None):
        if I is None:
            I = ChuggSimulator.default_I
        if wheels is None:
            wheels = ChuggSimulator.default_wheels

        self.I = np.array(I)

        #####################################################
        # Set up wheel data##################################
        #####################################################
        self.n = len(wheels)
        self.wheels = []

        for wheel in wheels:
            
            t = np.array(wheel['pos'])
            q = np.array(wheel['ori'])
            m = wheel['mass']
            name = wheel['name']
            ax = np.array(wheel['axis'])
            J = wheel['J']

            R = quat_to_rotation_matrix(q)
            U = translation_to_inertial_matrix(t)

            total_inertia = R.dot(J).dot(R.transpose()) + m*U

            at = quat_between(np.array((1.0,0.0,0.0)), ax)
            
            w = Wheel(t=t, q=q, m=m, name=name, axis=ax, J=J, R=R, U=U, I=total_inertia, axis_transform=at)
            
            self.wheels.append(w)

        self.static_wheel_inertia = np.sum([w.I for w in self.wheels], 0)

        #####################################################
        # State##############################################
        #####################################################
        
        self.ori = np.array((0.0,0.0,0.0,1.0))
        self.vel = np.array((0.0,0.0,0.0))
        self.wheel_vel = np.zeros(len(wheels))
        self.wheel_pos = np.zeros(len(wheels))
        
    def step(self, wheel_acc, ext_torque = None, dt = 0.02):
        if len(wheel_acc) != self.n:
            raise ValueError('Wrong number of reaction wheels.')
        
        if ext_torque is None:
            ext_torque = np.array((0.0,0.0,0.0))
        else:
            ext_torque = np.array(ext_torque, dtype=ChuggSimulator.dtype)
            # print ext_torque
            
        w = self.vel
        O = self.wheel_vel
        Odot = np.array(wheel_acc, dtype=ChuggSimulator.dtype)

        total_wheel_momentum = np.zeros(3)
        total_wheel_torque = np.zeros(3)
        
        for (wheel, o, odot) in zip(self.wheels, O, Odot):
            v = wheel.axis*o
            vdot = wheel.axis*odot
            # print w.I, v
            total_wheel_momentum += wheel.I.dot(v)
            total_wheel_torque += wheel.I.dot(vdot)

        

        # Total angular momentum
        M = self.I.dot(w) + self.static_wheel_inertia.dot(w) + total_wheel_momentum
        
        # Angular acceleration
        
        wdot = matrix_inverse(self.I + self.static_wheel_inertia).dot(ext_torque - total_wheel_torque - np.cross(w, M))

        # Integrate
        if np.linalg.norm(self.vel) != 0.0:
            theta = np.linalg.norm(self.vel)*dt
            axis = self.vel / np.linalg.norm(self.vel)
            deltaq = axisangle_to_quat(axis, theta)
            # print h, wdot
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
            wheel_vel = np.array(wheel_vel, dtype=ChuggSimulator.dtype)
        if wheel_pos == None:
            wheel_pos = np.zeros(len(self.wheels))
        else:
            wheel_pos = np.array([x % (2*np.pi) for x in wheel_pos], 
                                 dtype=ChuggSimulator.dtype)
        
        self.ori = np.array(ori, dtype=ChuggSimulator.dtype)
        self.vel = np.array(vel, dtype=ChuggSimulator.dtype)
        self.wheel_vel = wheel_vel
        self.wheel_pos = wheel_pos
