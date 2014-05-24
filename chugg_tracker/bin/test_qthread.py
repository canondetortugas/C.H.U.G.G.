#!/usr/bin/env python

import sys

from PyQt4 import QtCore, QtGui
from PyQt4.QtCore import pyqtSignal, pyqtSlot, QObject

import pyqtgraph as pg

import rospy

import tf.transformations as trans
from chugg_tracker.msg import Posterior

import numpy as np

channels = ['roll', 'pitch', 'yaw', 'wx', 'wy', 'wz']

class HistViewer:
    
    def __init__(self, names):
        self.nd = len(names)
        self.plots = {}
        self.win = pg.GraphicsWindow()
        for (idx, name) in zip(range(len(names)), names):
            pp = pg.PlotItem()
            pp.setTitle(name)
            self.win.addItem(pp, row=idx, col=0)
            self.plots[name] = pp
        
    def updateHist(self, name, x, y):
        plot = self.plots[name]
        plot.clear()
        # Pull out all left bin edges into x0 and right bin edges into x1
        x0 = x[:-1]
        x1 = x[1:]
        bb = pg.BarGraphItem(x0=x0, x1=x1, height=y)
        plot.addItem(bb)

    @pyqtSlot(dict)
    def updateAll(self, labeled_data):
        print "Updating all"
        for (name, data) in labeled_data.iteritems():
            self.updateHist(name, data[0], data[1])


        
class RosNode(QtCore.QThread):
    notify = pyqtSignal(dict)
    
    def __init__(self, viewer):
        QtCore.QThread.__init__(self)
        self.notify.connect(viewer.updateAll)

    def run(self):
        self.sub = rospy.Subscriber(rospy.get_param('~post_topic', 'chugg_tracker/filter/posterior'), 
                                    Posterior, callback=self.postCallback)

        rospy.spin()
    
    def postCallback(self, msg):
        print "Got callback"
        # samples = {'roll': [], 'pitch': [], 'yaw': [],
        #            'wx': [], 'wy': [], 'wz': []}
        samples = {channel: [] for channel in channels}
        
        hist = {}

        for sample in msg.samples:
            q = sample.ori
            v = sample.vel
            # I belive the 's' is for 'static'. If not, try 'r'
            r, p, y = trans.euler_from_quaternion((q.x, q.y, q.z, q.w), 'sxyz')
            samples['roll'].append(r)
            samples['pitch'].append(p)
            samples['yaw'].append(y)
            samples['wx'].append(v.x)
            samples['wy'].append(v.y)
            samples['wz'].append(v.z)
        for (ax, ss) in samples.iteritems():
            # bins is a list of the histogram bin edges (length(ss)+1)
            n, bins = np.histogram(ss, bins=100)
            hist[ax] = (bins, n)
        
        self.notify.emit(hist)

if __name__=="__main__":
    app = QtGui.QApplication(rospy.myargv())
    rospy.init_node('qthread_test')

    hv = HistViewer(channels)
    rn = RosNode(hv)

    rn.start()

    app.exec_()
