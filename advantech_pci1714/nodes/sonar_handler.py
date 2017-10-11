#!/usr/bin/env python
import rospy
import rosparam

import numpy as np
from scipy import optimize

#from sub8_msgs.srv import Sonar, SonarResponse

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from std_msgs.msg import UInt16
#from pinger_tracker.msg import *

from advantech_pci1714.srv import *

import time
import scipy.fftpack
import operator

class plotter():

    def location_service(self, data):

        #can change x1, x2, x3, y2, y3

        #MIL T-shape layout
        #hydro0_xyz = [0,      0,     0]
        #hydro1_xyz = [101.6,   0,     0]
        #hydro2_xyz = [-101.6,  0,     0]
        #hydro3_xyz = [0,  -101.6, 0]      

        # Equilateral layout (actual)
        hydro0_xyz = [0,      0,     0]
        hydro1_xyz = [101.6,   0,     0]
        hydro2_xyz = [-50.8,  88.0,     0]
        hydro3_xyz = [-50.8,  -88.0, 0]

        return Hydrophone_locations_serviceResponse(hydro0_xyz, hydro1_xyz, hydro2_xyz ,hydro3_xyz)

    def __init__(self):
        rospy.init_node('sonar_handler')

        rospy.Service('hydrophones/hydrophone_position', Hydrophone_locations_service, self.location_service)

        rate = rospy.Rate(10)

        #self.x = []
        self.a = []
        self.b = []
        self.c = []
        self.d = []
        self.trigger = False

        try:
            rospy.loginfo('waiting for hydrophones/ping service')
            rospy.wait_for_service('hydrophones/ping', timeout = 2)
            rospy.loginfo('detected hydrophones/ping service')
        except rospy.ROSException:
            rospy.logwarn("The hydrophones/ping service never showed up!")
            rospy.signal_shutdown("ROSPy Shutdown")
        
        while not rospy.is_shutdown():

            ping_service = rospy.ServiceProxy('hydrophones/ping', Ping)

        
            ping = ping_service()

            channels = ping.channels
            samples = float(ping.samples/channels)
            #print samples
            sample_rate = ping.sample_rate
            #print sample_rate
            adc_bit = ping.adc_bit
            data = ping.data

            #print("data received from service")

            rate.sleep()

def main():
    rospy.init_node('sonar_handler', anonymous=False)

    plotter()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print "Shutting down"
        pass
    


if __name__ == '__main__':
    main()