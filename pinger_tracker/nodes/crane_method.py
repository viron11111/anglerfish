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
import math

from dynamic_reconfigure.server import Server
from pinger_tracker.cfg import SignalConfig

#1600, -1600, -2000 (mm)
#0.0, 9.119770766119473, -9.016221156343818, -9.016221156343818

class solver():

    def crane_solver(self, data):

        c = 1.481 # speed of sound in 20 C water per uSec

        #position of hydrophones
        x1 = -25.4  #in mm
        x2 = 25.4
        y2 = 0
        x3 = 0
        y3 = -25.4

        #convert timestampts to distances
        #print data.calculated_time_stamps

        del1 = (data.calculated_time_stamps[1])*c #mm/uSec
        del2 = (data.calculated_time_stamps[2])*c #mm/uSec
        del3 = (data.calculated_time_stamps[3])*c #mm/uSec

        if del2 == 0:
            left = 0
            B1 = 0
        else:
            left = x2/del2
            B1 = -y2/del2

        if del1 == 0:
            right = 0
        else:
            right = x1/del1


        A1 = left - right    # eqn (12)
         

        if del1 == 0:
            catch = 0
        else:
            catch = (x1*x1-del1*del1)/(2*del1)

        if del2 == 0:
            zero = 0
        else:
            zero = (x2*x2 + y2*y2-del2*del2)/(2*del2)

        D1 = zero - catch

        A2 = x1/del1 - x3/del3    # eqn (14)
        B2 = -y3/del3 
        D2 = (x3*x3 + y3*y3-del3*del3)/(2.0*del3) - (x1*x1-del1*del1)/catch 

        x =  (B1*D2-B2*D1)/(A1*B2-A2*B1)   # eqn (15)
        y = -(A1*D2-A2*D1)/(A1*B2-A2*B1) 

        myx = x 
        myy = y        

        T1 = -4*del1*del1
        T2 =  4*(x1*x1-del1*del1)*myx*myx + 4*x1*(del1*del1-x1*x1)*myx + del1*del1*del1*del1 -2*del1*del1*x1*x1 -4*del1*del1*myy*myy + x1*x1*x1*x1

        zsquared = -T2/T1
        #print zsquared
        z = -math.sqrt(abs(zsquared))

        self.crane_pub.publish(Crane_pos(
            header=Header(stamp=rospy.Time.now(),
                          frame_id='Crane_pos'),
            x_pos=x,
            y_pos=y,
            z_pos=z))

        

    def __init__(self):
        rospy.init_node('crane_method')
        rospy.Subscriber('/hydrophones/calculated_time_stamps', Calculated_time_stamps, self.crane_solver)
        self.crane_pub = rospy.Publisher('hydrophones/crane_pos', Crane_pos, queue_size = 1)

        rate = rospy.Rate(1)

        #while not rospy.is_shutdown():

        #    rate.sleep()

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