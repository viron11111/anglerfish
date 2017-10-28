#!/usr/bin/env python
import rospy
import rosparam
import random
import numpy as np
import matplotlib.pyplot as plt
import time

from std_msgs.msg import Header
from pinger_tracker.msg import *
from advantech_pci1714.msg import *
from multilateration import Multilaterator, ReceiverArraySim, Pulse

import time
import sys
import csv
import os
from time import localtime,strftime
import datetime

class phaser(Multilaterator):

    def parse_ping(self, data):
        self.bit = data.adc_bit
        #timestamps = data.actual_time_stamps
        self.wave = [None]*len(data.data)
        samples = data.samples/4
        print samples
        for i in range(4):
            self.wave[i] = data.data[i::4]

        channel_length = len(self.wave[0])
        signed = [-(2**self.bit)/2]*channel_length

        #for i in range(4):
        #    self.wave[i] = [x + y for x, y in zip(self.wave[i], signed)]

        rospy.loginfo(self.wave[0][0])
        
        date_time = strftime("%d_%m_%y_%H_%M_%S", localtime())
        file_name = "/home/andy/catkin_ws/src/anglerfish/pinger_tracker/data/actual_signals_%s.csv" % date_time

        self.file = csv.writer(open(file_name,'w'))
        #self.file.writerow(["Actual time stamps"])
        self.file.writerow(["hydrophone 0", "hydrophone 1", "hydrophone 2", "hydrophone 3"])
        for i in range(samples):
            self.file.writerow(["%0.9f" % self.wave[0][i], "%0.9f" % self.wave[1][i], "%0.9f" % self.wave[2][i], "%0.9f" % self.wave[3][i]])
        
        #for i in range(4):
        #    self.file.writerow(["Hydrophone %i" % i])
        #    self.file.writerow(self.wave[i])



        os.system("rosnode kill ping_plotter")

    
    def __init__(self):      
        
        rospy.init_node('ping_plotter')

        rospy.Subscriber('/hydrophones/pingmsg', Pingdata, self.parse_ping)

        rate = rospy.Rate(1)  #rate of signals, 5 Hz for Anglerfish

        while not rospy.is_shutdown():

            rate.sleep()


def main():
    rospy.init_node('ping_plotter', anonymous=False)

    phaser()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print "Shutting down"
        pass
    


if __name__ == '__main__':
    main()