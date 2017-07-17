#!/usr/bin/env python
import rospy
import rosparam
import random
import numpy as np
import matplotlib.pyplot as plt

from std_msgs.msg import Header
from pinger_tracker.msg import *
from multilateration_do_not_use import Multilaterator, ReceiverArraySim, Pulse

import sys

from dynamic_reconfigure.server import Server
from pinger_tracker.cfg import SignalConfig



#*********************************************************************
#  Using mix of uSec and sec, pay attention to units
#  Hydrophone locations are in mm, along with speed of sound

class solver():

        

    def __init__(self):
        rospy.init_node('crane_method')

        rate = rospy.Rate(1)

        while not rospy.is_shutdown():

            rate.sleep()

def main():
    rospy.init_node('crane_method', anonymous=False)

    solver()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print "Shutting down"
        pass
    


if __name__ == '__main__':
    main()              