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

    def hydrophone_locations(self, data):
        self.hydro0 = [data.hydro0_xyz[0],data.hydro0_xyz[1],data.hydro0_xyz[2]]
        self.hydro1 = [data.hydro1_xyz[0],data.hydro1_xyz[1],data.hydro1_xyz[2]]
        self.hydro2 = [data.hydro2_xyz[0],data.hydro2_xyz[1],data.hydro2_xyz[2]]
        self.hydro3 = [data.hydro3_xyz[0],data.hydro3_xyz[1],data.hydro3_xyz[2]]    

    def calc_vals (self, data):
        #print data
        x,y,z = self.crane_solver(data)

        print "calculated x: %0.3f y: %0.3f" % (x,y)

        self.crane_pub.publish(Crane_pos(
            header=Header(stamp=rospy.Time.now(),
                          frame_id='Crane_pos_calc'),
            x_pos=x,
            y_pos=y,
            z_pos=z))

    def actu_vals (self, data):
        x,y,z = self.crane_solver(data)

        #print "actual x: %0.3f y: %0.3f" % (x,y)

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

        #position of hydrophones
        x1 = self.hydro1[0]  #in mm
        x2 = self.hydro2[0]
        y2 = self.hydro2[1]
        x3 = self.hydro3[0]
        y3 = self.hydro3[1]

        #convert timestampts to distances
        #print data.calculated_time_stamps
        if data.header.frame_id == "signal_sim":
            del1 = (data.actual_time_stamps[1])*c #mm/uSec
            del2 = (data.actual_time_stamps[2])*c #mm/uSec
            del3 = (data.actual_time_stamps[3])*c #mm/uSec
        elif data.header.frame_id == "phase_shift":
            del1 = (data.calculated_time_stamps[1])*c #mm/uSec
            del2 = (data.calculated_time_stamps[2])*c #mm/uSec
            del3 = (data.calculated_time_stamps[3])*c #mm/uSec  
            print "\n"
            print "calc1: %f calc2: %f, calc3: %f" % (data.calculated_time_stamps[1], data.calculated_time_stamps[2], data.calculated_time_stamps[3]) 
            print "del1: %f del2: %f del3 %f" % (del1,del2,del3)

        left = x1/del1 if del1 != 0 else 0
        B1 = -y2/del2 if del2 != 0 else 0
        right = x2/del2 if del2 != 0 else 0

        A1 = left - right    # eqn (12)
        
        catch = (x1*x1-del1*del1)/(2.0*del1) if del1 != 0 else 0
        zero = (x2*x2 + y2*y2-del2*del2)/(2.0*del2) if del2 != 0 else 0

        D1 = zero - catch
        
        zz = (x1/del1) if del1 != 0 else 0
        z = (x3/del3) if del3 != 0 else 0

        A2 = zz - z   # eqn (14)

        B2 = -y3/del3 if del3 != 0 else 0

        holder = (x3*x3 + y3*y3-del3*del3)/(2.0*del3) if del3 != 0 else 0
        holder2 = (x1*x1-del1*del1)/(2*del1) if del1 != 0 else 0

        D2 = holder - holder2 


        x =  (B1*D2-B2*D1)/(A1*B2-A2*B1) if (A1*B2-A2*B1) != 0 else 0  # eqn (15)
        y = -(A1*D2-A2*D1)/(A1*B2-A2*B1) if (A1*B2-A2*B1) != 0 else 0

        print "A1: %f" % A1
        print "D1: %f" % D1
        print "A2: %f" % A2
        print "B2: %f" % B2
        print "D2: %f" % D2

        #print x
        #print y

        myx = x 
        myy = y        

        T1 = -4*del1*del1
        T2 =  4*(x1*x1-del1*del1)*myx*myx + 4*x1*(del1*del1-x1*x1)*myx + del1*del1*del1*del1 -2*del1*del1*x1*x1 -4*del1*del1*myy*myy + x1*x1*x1*x1

        zsquared = -T2/T1 if T1 != 0 else 0

        z = -math.sqrt(abs(zsquared))

        Pz = z
        Px = x
        Py = y



        return (x, y, z)

        check_d0 = math.sqrt(Px*Px+Py*Py+Pz*Pz) 
        check_d1 = math.sqrt((Px-x1)*(Px-x1)+Py*Py+Pz*Pz) 
        check_d2 = math.sqrt((Px-x2)*(Px-x2)+(Py-y2)*(Py-y2)+Pz*Pz) 
        check_d3 = math.sqrt((Px-x3)*(Px-x3)+(Py-y3)*(Py-y3)+Pz*Pz) 

        print "Given timestamps:\t"
        print "del1 = %0.5f del2 = %0.5f del3 = %0.5f" % (data.actual_time_stamps[1:])
        print "Re-calculated timestamps:\t"
        print "del1 = %0.5f del2 = %0.5f del3 = %0.5f" % (check_d1-check_d0, check_d2-check_d0, check_d3-check_d0)
        print "\n"

        

    def __init__(self):
        rospy.init_node('crane_method')
        #rospy.Subscriber('/hydrophones/actual_time_stamps', Actual_time_stamps, self.actu_vals)
        rospy.Subscriber('/hydrophones/calculated_time_stamps', Calculated_time_stamps, self.calc_vals)
        rospy.Subscriber('hydrophones/hydrophone_locations', Hydrophone_locations, self.hydrophone_locations)

        self.crane_serv = rospy.Service('hydrophones/crane_srv', Crane_solution, self.crane_solutions)
        self.crane_pub = rospy.Publisher('hydrophones/crane_pos', Crane_pos, queue_size = 1)

        self.hydro0 = [0,0,0]
        self.hydro1 = [0,0,0]
        self.hydro2 = [0,0,0]
        self.hydro3 = [0,0,0]         

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