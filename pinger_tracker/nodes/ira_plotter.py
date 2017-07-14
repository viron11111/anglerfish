#!/usr/bin/env python
import rospy
import rosparam
import random
import numpy as np
import matplotlib.pyplot as plt
import time

from std_msgs.msg import Header
from pinger_tracker.msg import *
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
        timestamps = data.actual_time_stamps
        self.wave = [None]*len(data.data)
        for i in range(4):
            self.wave[i] = data.data[i::4]

        channel_length = len(self.wave[0])
        signed = [-(2**self.bit)/2]*channel_length

        for i in range(4):
            self.wave[i] = [x + y for x, y in zip(self.wave[i], signed)]

        
        date_time = strftime("%d_%m_%y_%H_%M_%S", localtime())
        file_name = "/home/andy/catkin_ws/src/anglerfish/pinger_tracker/data/sim_signal_%s.csv" % date_time

        self.file = csv.writer(open(file_name,'w'))
        self.file.writerow(["Actual time stamps"])
        self.file.writerow(["hydrophone 0", "hydrophone 1", "hydrophone 2", "hydrophone 3"])
        self.file.writerow(["%0.9f" % timestamps[0], "%0.9f" % timestamps[1], "%0.9f" % timestamps[2], "%0.9f" % timestamps[3]])
        for i in range(4):
            self.file.writerow(["Hydrophone %i" % i])
            self.file.writerow(self.wave[i])



        os.system("rosnode kill ping_plotter")

    
    def __init__(self):      
        
        rospy.init_node('ping_plotter')

        rospy.Subscriber('/hydrophones/ping', Ping, self.parse_ping)

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