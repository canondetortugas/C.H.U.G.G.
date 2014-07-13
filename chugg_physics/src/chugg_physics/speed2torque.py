# -*- coding: utf-8 -*-
"""
Created on Tue Jun 24 15:45:49 2014

@author: turner

"""


from pylab import *

# def speed2torque(a):
#     b=-(7*sqrt(1389*(a-474.678))-10798)/(68061);
#     return b;

# def fitpoly2(x):
#     p1 =   3.441e-09
#     p2 =   -6.68e-05
#     p3 =      0.3945
#     return p1*x**2 + p2*x + p3

# def fitpoly3(x):
#     p1 =  -6.248e-13
#     p2 =   1.653e-08
#     p3 =  -0.0001524  
#     p4 =       0.569
#     return  p1*x**3 + p2*x**2 + p3*x + p4

# def speed2torque(x):


def speed2torque(x):
    '''Map the current speed [rad/s] of a Maxon EC 45 Flat motor to 
    maximum torque available [N*m]'''
    assert x >= 0 and x <= 10000*2*pi/60.0
  
    stall_torque = 0.82
    
    if x > 0:
        torque = -0.109*log(x) + 0.8195
    else:
        torque = stall_torque
    
    return min(stall_torque, torque)
