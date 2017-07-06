#!/usr/bin/env python
import rospy
import rosparam

import numpy as np
from scipy import optimize

from sub8_msgs.srv import Sonar, SonarResponse

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from std_msgs.msg import UInt16
from pinger_tracker.msg import *

import time

class plotter():

    def signal_plotter(self, data):
        #print "msg received"
        values = data.data
        print "msg"

        self.x = np.arange(len(values[:300:4]))
        self.a = values[0:300:4]
        self.b = values[1:300:4]
        self.c = values[2:300:4]
        self.d = values[3:300:4]

        self.trigger = True
        #s = 1 + np.sin(2*np.pi*t)


    def __init__(self):
        rospy.init_node('ping_plotter')
        rospy.Subscriber('/hydrophones/ping', Ping, self.signal_plotter)

        rate = rospy.Rate(5)

        self.x = []
        self.a = []
        self.b = []
        self.c = []
        self.d = []
        self.trigger = False

        while not rospy.is_shutdown():
            print self.trigger
            if self.trigger == True:
                plt.gcf().clear()
                plt.plot(self.x, self.a)
                plt.plot(self.x, self.b)
                plt.plot(self.x, self.c)
                plt.plot(self.x, self.d)

                plt.xlabel('time (uS)')
                plt.ylabel('voltage (mV)')
                plt.title('X')
                plt.grid(True)
                plt.show()
                self.trigger = False

            rate.sleep()

        plt.close('all')

def main():
    rospy.init_node('ping_plotter', anonymous=False)

    plotter()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print "Shutting down"
        pass
    


if __name__ == '__main__':
    main()