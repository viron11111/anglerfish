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
        samples = float(data.samples/4)
        sample_rate = float(data.sample_rate)
        time = (samples/sample_rate)*10**6
        #print time

        #print "msg"

        length = 256
        starting_sample = 0
        distance = (length/samples)*time
        '''print time
        print distance
        print time/samples
        print time/(time/samples)'''

        self.x = np.arange(starting_sample*(time/samples),distance, time/samples)
        #print self.x
        self.a = values[starting_sample*4 + 0:length*4:4]
        #print self.a
        self.b = values[starting_sample*4 + 1:length*4:4]
        self.c = values[starting_sample*4 + 2:length*4:4]
        self.d = values[starting_sample*4 + 3:length*4:4]

        self.trigger = True
        #s = 1 + np.sin(2*np.pi*t)


    def __init__(self):
        rospy.init_node('signal_simulator')

        self.sample_rate = rospy.get_param('~sample_rate', 300e3)
        self.thresh = rospy.get_param('~thresh', 500)
        self.frame = rospy.get_param('~frame', '/hydrophones')
        permute_str = rospy.get_param('~permute', '1 2 3 4')
        self.samples = rospy.get_param('sample_number', 1024)
        
        self.signal_pub = rospy.Publisher('/hydrophones/ping', Ping, queue_size = 1)

        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            
            rate.sleep()

def main():
    rospy.init_node('signal_simulator', anonymous=False)

    simulator()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print "Shutting down"
        pass
    


if __name__ == '__main__':
    main()