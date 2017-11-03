#!/usr/bin/env python
import rospy
import rosparam
import random
import numpy as np
import matplotlib.pyplot as plt

from std_msgs.msg import Header
from pinger_tracker.msg import *
#from multilateration import Multilaterator, ReceiverArraySim, Pulse

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

    def calc_vals(self, data):

        P1 = [0,0,0]
        P2 = [0,0,0]

        c = 1.484 # speed of sound in 20 C water per uSec
        #c = 0.343 #speed of sound in air

        '''hydro = rospy.ServiceProxy('hydrophones/hydrophone_position', Hydrophone_locations_service)
        holder = hydro()

        self.hydro0 = holder.hydro0_xyz
        self.hydro1 = holder.hydro1_xyz
        self.hydro2 = holder.hydro2_xyz
        self.hydro3 = holder.hydro3_xyz       

        #position of hydrophones
        x1 = self.hydro1[0] #+ 0.75 #np.random.uniform(-1, 1)  #in mm
        x2 = self.hydro2[0] #- 0.75 #np.random.uniform(-1, 1)
        y2 = self.hydro2[1] #+ 0.75 #np.random.uniform(-1, 1)
        x3 = self.hydro3[0] #- 0.75 #np.random.uniform(-1, 1)
        y3 = self.hydro3[1] #+ 0.75 #np.random.uniform(-1, 1)
        z3 = self.hydro3[2]
        #print "x1: %f" % x1'''

        #MIL T-shape layout
        #hydro0_xyz = [0,      0,     0]
        #hydro1_xyz = [100.0,   0,     0]
        #hydro2_xyz = [-100.0,  0,     0]
        #hydro3_xyz = [0,  -100.0, 0]      

        # Equilateral layout (actual)
        hydro0_xyz = [0,      0,     0]
        hydro1_xyz = [100.0,   0,     0]
        hydro2_xyz = [-50,  86.6,     0]
        hydro3_xyz = [-50,  -86.6, 0]

        #experimental layout
        #hydro0_xyz = [0,      0,     0]
        #hydro1_xyz = [173.2,   0,     0]
        #hydro2_xyz = [86.6,  150,     0]
        #hydro3_xyz = [86.6,  50, 0]       

        x1 = hydro1_xyz[0] #+ 0.75 #np.random.uniform(-1, 1)  #in mm
        x2 = hydro2_xyz[0] #- 0.75 #np.random.uniform(-1, 1)
        y2 = hydro2_xyz[1] #+ 0.75 #np.random.uniform(-1, 1)
        x3 = hydro3_xyz[0] #- 0.75 #np.random.uniform(-1, 1)
        y3 = hydro3_xyz[1] #+ 0.75 #np.random.uniform(-1, 1)
        #z3 = hydro3_xyz[2]

        del1 = (data.calculated_time_stamps[1])*c #mm/uSec
        del2 = (data.calculated_time_stamps[2])*c #mm/uSec
        del3 = (data.calculated_time_stamps[3])*c #mm/uSec   

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

        myx = x 
        myy = y        

        T1 = -4*del1*del1
        T2 =  4*(x1*x1-del1*del1)*myx*myx + 4*x1*(del1*del1-x1*x1)*myx + del1*del1*del1*del1 -2*del1*del1*x1*x1 -4*del1*del1*myy*myy + x1*x1*x1*x1

        zsquared = -T2/T1 if T1 != 0 else 0

        z = -math.sqrt(abs(zsquared))

        Pz = z
        Px = x
        Py = y

        vector = [x,y,z]
        print vector


        #return (x, y, z)

        check_d0 = math.sqrt(Px*Px+Py*Py+Pz*Pz) 
        check_d1 = math.sqrt((Px-x1)*(Px-x1)+Py*Py+Pz*Pz) 
        check_d2 = math.sqrt((Px-x2)*(Px-x2)+(Py-y2)*(Py-y2)+Pz*Pz) 
        check_d3 = math.sqrt((Px-x3)*(Px-x3)+(Py-y3)*(Py-y3)+Pz*Pz) 

        #print "Given timestamps:\t"
        #print "del1 = %0.5f del2 = %0.5f del3 = %0.5f" % (data.actual_time_stamps[1:])
        #print "Re-calculated timestamps:\t"
        #print "del1 = %0.5f del2 = %0.5f del3 = %0.5f" % (check_d1-check_d0, check_d2-check_d0, check_d3-check_d0)
        #print "\n"
            
        self.crane_pub = rospy.Publisher('hydrophones/crane_pos', Crane_pos, queue_size = 1)
        self.crane_pub.publish(Crane_pos(
            header=Header(stamp=rospy.Time.now(),
                          frame_id='Crane_pos_calc'),
            x_pos=x,
            y_pos=-y,
            z_pos=z))            

        #print "x: %f, y: %f, z: %f" % (x,y,z)

        '''P1[0] = (-BB+math.sqrt(discr))/(2.0*AA) if (2.0*AA) != 0 else 0
        P2[0] = (-BB-math.sqrt(discr))/(2.0*AA) if (2.0*AA) != 0 else 0

        # get corresponding value for y coordinate  ;  eqn (11)
        P1[1] = (A1*P1[0]+D1)/B1 if B1 != 0 else 0
        P2[1] = (A1*P2[0]+D1)/B1 if B1 != 0 else 0

        # get correspoinding value for z coordinate ;  eqn (16)
        P1[2] = -(Q2a*P1[0]+Q2b)/Q1 if Q1 != 0 else 0
        P2[2] = -(Q2a*P2[0]+Q2b)/Q1 if Q1 != 0 else 0'''





        return Crane_pos_serviceResponse(x, y, z)        

    def __init__(self):
        rospy.init_node('crane_method_service')
        #rospy.Subscriber('/hydrophones/actualtimestamps', Actual_time_stamps, self.actu_vals)
        rospy.Subscriber('/hydrophones/calculated_time_stamps', Calculated_time_stamps, self.calc_vals)
        #rospy.Subscriber('hydrophones/hydrophone_locations', Hydrophone_locations, self.hydrophone_locations)

        #self.crane_serv = rospy.Service('hydrophones/crane_srv', Crane_pos_service, self.crane_solver)
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

    '''

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

        z = -math.sqrt(abs(zsquared))'''
