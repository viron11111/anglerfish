#!/usr/bin/env python
import rospy
import rosparam
import random
import numpy as np
import matplotlib.pyplot as plt

from std_msgs.msg import Header
from pinger_tracker.msg import *
from multilateration import Multilaterator, ReceiverArraySim, Pulse

import sys
import math

from dynamic_reconfigure.server import Server
from pinger_tracker.cfg import SignalConfig
from pinger_tracker.srv import *

#1600, -1600, -2000 (mm)
#0.0, 9.119770766119473, -9.016221156343818, -9.016221156343818

class solver():

    def calc_vals (self, data):
        x,y,z = self.crane_solver(data)

        self.crane_pub.publish(Crane_pos(
            header=Header(stamp=rospy.Time.now(),
                          frame_id='Crane_pos_calc'),
            x_pos=x,
            y_pos=y,
            z_pos=z))

    def actu_vals (self, data):
        x,y,z = self.crane_solver(data)

        self.crane_pub.publish(Crane_pos(
            header=Header(stamp=rospy.Time.now(),
                          frame_id='Crane_pos_actu'),
            x_pos=x,
            y_pos=y,
            z_pos=z))        

    def crane_solutions(self, data):
        rospy.loginfo("service response crane_srv")
        x = 3000
        y = 1000
        z = 2000
        return x,y,z

    def crane_solver(self, data):

        c = 1.484 # speed of sound in 20 C water per uSec

        hydro = rospy.ServiceProxy('hydrophones/hydrophone_position', Hydrophone_locations_service)
        holder = hydro()

        self.hydro0 = holder.hydro0_xyz
        self.hydro1 = holder.hydro1_xyz
        self.hydro2 = holder.hydro2_xyz
        self.hydro3 = holder.hydro3_xyz        

        #position of hydrophones
        x1 = self.hydro1[0]  #in mm
        x2 = self.hydro2[0]
        y2 = self.hydro2[1]
        x3 = self.hydro3[0]
        y3 = self.hydro3[1]

        calc_service = rospy.ServiceProxy('hydrophones/calculated_time_stamps', Calculated_time_stamps_service)
        tstampts = calc_service()

        '''calc_service = rospy.ServiceProxy('hydrophones/actual_time_stamps', Actual_time_stamps_service)
        tstampts = calc_service()

        del1 = (tstampts.actual_time_stamps[1])*c #mm/uSec
        del2 = (tstampts.actual_time_stamps[2])*c #mm/uSec
        del3 = (tstampts.actual_time_stamps[3])*c #mm/uSec '''       

        del1 = (tstampts.calculated_time_stamps[1])*c #mm/uSec
        del2 = (tstampts.calculated_time_stamps[2])*c #mm/uSec
        del3 = (tstampts.calculated_time_stamps[3])*c #mm/uSec   

        #print "\n"
        #print "del1: %f del2: %f del3: %f" % (del1, del2, del3)        


        left = x1/del1 if del1 != 0 else 0
        B1 = -y2/del2 if del2 != 0 else 0
        right = x2/del2 if del2 != 0 else 0

        #print "left = %f B1 = %f right = %f" % (left, B1, right)

        A1 = left - right    # eqn (12)

        #print "A1: %f" % A1
        
        catch = (x1*x1-del1*del1)/(2.0*del1) if del1 != 0 else 0
        zero = (x2*x2 + y2*y2-del2*del2)/(2.0*del2) if del2 != 0 else 0

        D1 = zero - catch

        #print "D1: %f" % D1
        
        zz = (x1/del1) if del1 != 0 else 0
        z = (x3/del3) if del3 != 0 else 0

        A2 = zz - z   # eqn (14)

        #print "A2: %f" % A2

        B2 = -y3/del3 if del3 != 0 else 0

        #print "B2: %f" % B2

        holder = (x3*x3 + y3*y3-del3*del3)/(2.0*del3) if del3 != 0 else 0
        holder2 = (x1*x1-del1*del1)/(2*del1) if del1 != 0 else 0

        D2 = holder - holder2 

        #print "D2: %f" % D2

        x =  (B1*D2-B2*D1)/(A1*B2-A2*B1) if (A1*B2-A2*B1) != 0 else 0  # eqn (15)
        y = -(A1*D2-A2*D1)/(A1*B2-A2*B1) if (A1*B2-A2*B1) != 0 else 0

        myx = x 
        myy = y        

        T1 = -4*del1*del1
        T2 =  4*(x1*x1-del1*del1)*myx*myx + 4*x1*(del1*del1-x1*x1)*myx + del1*del1*del1*del1 -2*del1*del1*x1*x1 -4*del1*del1*myy*myy + x1*x1*x1*x1

        zsquared = -T2/T1 if T1 != 0 else 0

        z = -math.sqrt(abs(zsquared))

        #print "x: %f, y: %f, z: %f" % (x,y,z)

        return Crane_pos_serviceResponse(x, y, z)        

    def __init__(self):
        rospy.init_node('crane_method_service')
        #rospy.Subscriber('/hydrophones/actualtimestamps', Actual_time_stamps, self.actu_vals)
        #rospy.Subscriber('/hydrophones/calculated_time_stamps', Calculated_time_stamps, self.calc_vals)
        #rospy.Subscriber('hydrophones/hydrophone_locations', Hydrophone_locations, self.hydrophone_locations)

        self.crane_serv = rospy.Service('hydrophones/crane_srv', Crane_pos_service, self.crane_solver)
        #self.crane_pub = rospy.Publisher('hydrophones/crane_pos', Crane_pos, queue_size = 1)

        rate = rospy.Rate(1)

        while not rospy.is_shutdown():

            rate.sleep()

def main():
    
    solver()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print "Shutting down"
        pass
    


if __name__ == '__main__':
    main()              