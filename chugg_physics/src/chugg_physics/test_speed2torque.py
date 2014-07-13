#!/usr/bin/env python

from speed2torque import speed2torque

from pylab import *

if __name__=='__main__':
    x = linspace(0, 10000, 10000)*2*pi/60
    y = [speed2torque(xx) for xx in x]
    figure()
    plot(x, y)
    xlabel('Speed [rad/s]')
    ylabel('Available Torque [N*m]')
    show()
