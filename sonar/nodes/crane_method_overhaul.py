#!/usr/bin/env python
import rospy
import rosparam
import random
import numpy as np
import matplotlib.pyplot as plt
import operator

from std_msgs.msg import Header, Float32
from pinger_tracker.msg import *
from sonar.msg import Bearing
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

    def cardinal(self, del1, del2, del3):
        del0 = 0.0
        bearing = 0.0
        tolerance = 15 #15
        self.psolution = 0
        dels = {"del0": del0, "del1": del1, "del2": del2, "del3": del3}
        sorted_dels = sorted(dels.items(), key=operator.itemgetter(1))
        sorted_dels = (sorted_dels[0][0],sorted_dels[1][0],sorted_dels[2][0],sorted_dels[3][0])  
        self.sorted_dels = sorted_dels   
        if sorted_dels == ('del2', 'del3', 'del0', 'del1'): #double checked
            if abs(del1-del0) < tolerance:
                bearing = 90.0
            elif abs(del0-del3) < tolerance:
                bearing = 60.0
            else:
                bearing = 75.0  
            self.psolution = 1
        elif sorted_dels == ('del2', 'del3', 'del1', 'del0'): #double checked
            if abs(del1-del0) < tolerance:
                bearing = 90.0
            elif abs(del1-del3) < tolerance:
                bearing = 120
            else:
                bearing = 105     
            self.psolution = 1  #changed changed back
            #print "here1"  
        elif sorted_dels == ('del2', 'del1', 'del3', 'del0'): #double checked
            if abs(del1-del2) < tolerance:
                bearing = 150         
            elif abs(del1-del3) < tolerance:
                bearing = 120
            else:
                bearing = 135
            self.psolution = 1
        elif sorted_dels == ('del1', 'del2', 'del3', 'del0'): #double checked
            if abs(del3-del2) < tolerance:
                bearing = 180
            elif abs(del1-del2) < tolerance:
                bearing = 150
            else:                
                bearing = 165.0
            self.psolution = 1
        elif sorted_dels == ('del1', 'del3', 'del2', 'del0'): #switching between 1 and 2
            if abs(del2-del3) < tolerance:
                bearing = 180
            elif abs(del2-del0) < tolerance:
                bearing = 210
            else:                            
                bearing = 195
            self.psolution = 1
            #print "here2" 
        elif sorted_dels == ('del1', 'del3', 'del0', 'del2'): #double checked
            if abs(del2) < tolerance:
                bearing = 210
            elif abs(del3-del0)<tolerance:
                bearing = 240
            else:
                bearing = 225
            self.psolution = 2
            #print "here3" 
        elif sorted_dels == ('del1', 'del0', 'del3', 'del2'): #double checked
            if abs(del1-del0) < tolerance:
                bearing = 270.0
            elif abs(del0-del3) < tolerance:
                bearing = 240.0
            else:
                bearing = 255.0
            self.psolution = 1
        elif sorted_dels == ('del0', 'del1', 'del3', 'del2'):#double checked
            if abs(del1-del0) < tolerance:
                bearing = 270.0
            elif abs(del3-del1) < tolerance:
                bearing = 300.0
            else:
                bearing = 285.0
            self.psolution = 2
            #print "here4" 
        elif sorted_dels == ('del0', 'del3', 'del1', 'del2'):#double checked
            if abs(del1-del2) < tolerance:
                bearing = 330
            elif abs(del1-del3) < tolerance:
                bearing = 300
            else:                            
                bearing = 315 
            self.psolution = 1
        elif sorted_dels == ('del0', 'del3', 'del2', 'del1'):#double checked 
            if abs(del1-del2) < tolerance:
                bearing = 330.0
            elif abs(del2-del3) < tolerance:
                bearing = 0.0
            else:                            
                bearing = 345.0
            self.psolution = 2
            #print "here5" 
        elif sorted_dels == ('del0', 'del2', 'del3', 'del1'):#double checked             
            if abs(del2-del3) < tolerance:            
                bearing = 0.0
            elif abs(del2-del0) < tolerance:
                bearing = 30.0
            else:
                bearing = 15.0  
            self.psolution = 2  
        elif sorted_dels == ('del2', 'del0', 'del3', 'del1'):#double checked 
            if abs(del0-del3) < tolerance:
                bearing = 60.0
            if abs(del0-del2) < tolerance:
                bearing = 30.0
            else:
                bearing = 45.0
            self.psolution = 2
            #print "here6" 
        else:
            rospy.logerr("CARDINAL failed to find solution!")
            bearing = -1

        if sorted_dels[3] == 'del0':
            self.ref_hydro = 0
        elif sorted_dels[3] == 'del1':
            self.ref_hydro = 1
        elif sorted_dels[3] == 'del2':
            self.ref_hydro = 2

        self.cardinal_pub = rospy.Publisher('hydrophones/cardinal', Float32, queue_size = 1)
        self.cardinal_pub.publish(Float32(bearing))


        #rospy.logerr(bearing)       
        return bearing
        
        #print sorted_dels
    def pol2cart(self, rho, phi):
        x = rho * np.cos(phi)
        y = rho * np.sin(phi)
        return(x, y)        

    def calc_vals(self, data):

        P1 = [0,0,0]
        P2 = [0,0,0]

        c = 1.484 # speed of sound in 20 C water per uSec
        #c = 0.343 #speed of sound in air     

        #experimental layout

        #hydro0_xyz = [0,      0,     0]
        #hydro1_xyz = [-173.2,   0,     0]
        #hydro2_xyz = [-86.6,  -150,     0]
        #hydro3_xyz = [-86.6,  -50, -100] 

        #hydro0_xyz = [0,      0,     0]
        #hydro1_xyz = [-168,   0,     0]
        #hydro2_xyz = [-86.6,  -149,     0]
        #hydro3_xyz = [-86.6,  -49, -100]  

        #experimental layout
             

        del1i = (data.calculated_time_stamps[1])*c #mm/uSec
        del2i = (data.calculated_time_stamps[2])*c #mm/uSec
        del3i = (data.calculated_time_stamps[3])*c #mm/uSec         

        bearing = self.cardinal(data.calculated_time_stamps[1],data.calculated_time_stamps[2],data.calculated_time_stamps[3])

        #del1i = (data.actual_time_stamps[1])*c #mm/uSec
        #del2i = (data.actual_time_stamps[2])*c #mm/uSec
        #del3i = (data.actual_time_stamps[3])*c #mm/uSec 

        print "del1: %0.10f del2: %0.10f del3: %0.10f" % (del1i, del2i, del3i)        

        #bearing = self.cardinal(data.actual_time_stamps[1],data.actual_time_stamps[2],data.actual_time_stamps[3])

        #if self.ref_hydro == 0:
        #calibrated data *save**
        #hydro0_xyz = [0,      0,     0]
        #hydro1_xyz = [-172.2,   0,     0]
        #hydro2_xyz = [-86.6,  -146,     0]
        #hydro3_xyz = [-86.6,  -53, -100]   

        hydro0_xyz = [0,      0,     0]
        hydro1_xyz = [-173.2,   0,     0]
        hydro2_xyz = [-86.6,  -150,     0]
        hydro3_xyz = [-86.6,  -50, -100]
        del1 = del1i
        del2 = del2i
        del3 = del3i 
        #elif self.ref_hydro == 1:
        #    del1 = 



        x1 = hydro1_xyz[0] #+ 0.75 #np.random.uniform(-1, 1)  #in mm
        x2 = hydro2_xyz[0] #- 0.75 #np.random.uniform(-1, 1)
        y2 = hydro2_xyz[1] #+ 0.75 #np.random.uniform(-1, 1)
        x3 = hydro3_xyz[0] #- 0.75 #np.random.uniform(-1, 1)
        y3 = hydro3_xyz[1] #+ 0.75 #np.random.uniform(-1, 1)
        z3 = hydro3_xyz[2]            


        print "********"

        left = x1/del1 if del1 != 0 else 0
        right = x2/del2 if del2 != 0 else 0

        #print "left = %f B1 = %f right = %f" % (left, B1, right)

        A1 = left - right    # eqn (12)
        B1 = -y2/del2 if del2 != 0 else 0

        #print "A1: %f" % A1
        
        zero = (x2*x2 + y2*y2-del2*del2)/(2.0*del2) if del2 != 0 else 0
        catch = (x1*x1-del1*del1)/(2.0*del1) if del1 != 0 else 0

        D1 = zero - catch

        #print "D1: %f" % D1
        
        Q1 = -z3/del3 if del3 != 0 else 0

        Q2aleft = (A1*y3)/(B1*del3) if (B1*del3) !=0 else 0
        Q2amiddle = x3/del3 if del3 != 0 else 0
        Q2aright = x1/del1 if del1 != 0 else 0

        Q2a = Q2aleft - Q2amiddle + Q2aright

        Q2bleft = (del1*del1-x1*x1)/(2.0*del1) if (2.0*del1) != 0 else 0
        Q2bmiddle = (D1*y3)/(B1*del3) if (B1*del3) != 0 else 0
        Q2bright = (del3*del3 - x3*x3 - y3*y3 - z3*z3)/(2.0*del3) if (2.0*del3) != 0 else 0

        Q2b = Q2bleft + Q2bmiddle - Q2bright

        R1aleft = (A1*A1)/(B1*B1) if (B1*B1) != 0 else 0
        R1aright = (x1*x1)/(del1*del1) if (del1*del1) != 0 else 0

        R1a = R1aleft + 1 - R1aright

        R1bleft = (x1*x1*x1)/(del1*del1) if (del1*del1) != 0 else 0
        R1bright = (2.0*A1*D1)/(B1*B1) if (B1*B1) != 0 else 0

        R1b = R1bleft - x1 + R1bright

        R1cleft = (D1*D1)/(B1*B1) if (B1*B1) != 0 else 0
        R1cright = (x1*x1*x1*x1)/(4.0*del1*del1) if (4.0*del1*del1) != 0 else 0

        R1c = (x1*x1)/2.0 - (del1*del1)/4.0 + R1cleft - R1cright

        AA = Q1*Q1*R1a + Q2a*Q2a
        BB = Q1*Q1*R1b + 2.0*Q2a*Q2b
        CC = Q1*Q1*R1c + Q2b*Q2b

        discr = BB*BB - 4.0*AA*CC ;

        phi = math.radians(360-bearing)
        rho = 500.0


        if (discr < 0):
            rospy.loginfo("no real solution was found; set garbage values for P1 and P2")

            print discr

            (x,y) = self.pol2cart(rho,phi)

            P1[0] = P1[1] = P1[2] = 0.0
            P2[0] = P2[1] = P2[2] = 0.0
            z=0         

        else:

            P1[0] = (-BB+math.sqrt(discr))/(2.0*AA) if (2.0*AA) != 0 else 0
            P2[0] = (-BB-math.sqrt(discr))/(2.0*AA) if (2.0*AA) != 0 else 0

            # get corresponding value for y coordinate  ;  eqn (11)
            P1[1] = -(A1*P1[0]+D1)/B1 if B1 != 0 else 0
            P2[1] = -(A1*P2[0]+D1)/B1 if B1 != 0 else 0

            # get correspoinding value for z coordinate ;  eqn (16)
            P1[2] = -(Q2a*P1[0]+Q2b)/Q1 if Q1 != 0 else 0
            P2[2] = -(Q2a*P2[0]+Q2b)/Q1 if Q1 != 0 else 0

            d0_1 = -P1[0] * (x1 / del1) + (x1*x1 - del1*del1) / (2.0*del1)        
            d0_2 = -P2[0] * (x1 / del1) + (x1*x1 - del1*del1) / (2.0*del1)

            rospy.loginfo("x1: %f" % (P1[0]))
            rospy.loginfo("y1: %f" % (P1[1]))
            rospy.loginfo("z1: %f" % (P1[2]))
            rospy.loginfo("**")
            rospy.loginfo("x2: %f" % (P2[0]))
            rospy.loginfo("y2: %f" % (P2[1]))
            rospy.loginfo("z2: %f" % (P2[2]))

            dellist = [del1, del2, del3]

            check_d0 = math.sqrt(P1[0]*P1[0]+P1[1]*P1[1]+P1[2]*P1[2])
            check_d1 = math.sqrt((P1[0]-x1)*(P1[0]-x1)+P1[1]*P1[1]+P1[2]*P1[2])
            check_d2 = math.sqrt((P1[0]-x2)*(P1[0]-x2)+(P1[1]-y2)*(P1[1]-y2)+P1[2]*P1[2])
            check_d3 = math.sqrt((P1[0]-x3)*(P1[0]-x3)+(P1[1]-y3)*(P1[1]-y3)+(P1[2]-z3)*(P1[2]-z3))

            measured1_list = [check_d1 - check_d0, check_d2 - check_d0, check_d3 - check_d0]

            check_d0 = math.sqrt(P2[0]*P2[0]+P2[1]*P2[1]+P2[2]*P2[2])
            check_d1 = math.sqrt((P2[0]-x1)*(P2[0]-x1)+P2[1]*P2[1]+P2[2]*P2[2])
            check_d2 = math.sqrt((P2[0]-x2)*(P2[0]-x2)+(P2[1]-y2)*(P2[1]-y2)+P2[2]*P2[2])
            check_d3 = math.sqrt((P2[0]-x3)*(P2[0]-x3)+(P2[1]-y3)*(P2[1]-y3)+(P2[2]-z3)*(P2[2]-z3))

            measured2_list = [check_d1 - check_d0, check_d2 - check_d0, check_d3 - check_d0]

            measured1_list = [int(i) for i in measured1_list]
            measured2_list = [int(i) for i in measured2_list]
            dellist = [int(i) for i in dellist]

            print "dellist:        ", dellist
            print "measured1_list: ", measured1_list
            print "measured2_list: ", measured2_list

            x=0
            y=0
            z=0           

            p1_heading = np.arctan2(P1[0],P1[1]) - np.pi/2
            p1_heading = math.degrees(p1_heading)
            p2_heading = np.arctan2(P2[0], P2[1]) - np.pi/2
            p2_heading = math.degrees(p2_heading)

            if p1_heading < 0:
                p1_heading = 360 + p1_heading
            if p2_heading < 0:
                p2_heading = 360 + p2_heading                

            rospy.loginfo("p1_heading: %0.2f" % p1_heading)
            rospy.loginfo("p2_heading: %0.2f" % p2_heading)

            inv_bearing = bearing + 180   
            if inv_bearing > 360:
                inv_bearing = inv_bearing - 360         
            #inv_p1heading = p1_heading + 180
            #inv_p2heading = p2_heading + 180


            #rospy.logwarn("do_1: %0.2f" % d0_1)
            #rospy.logwarn("do_2: %0.2f" % d0_2)

            #*********************************************

            #corrections using dels lookup table.
            #rospy.logwarn(self.psolution)
            #map(abs, myList)

            angle_tolerance = 45

            if map(abs,measured1_list) == map(abs,dellist) or map(abs,measured2_list) == map(abs,dellist):

                if measured1_list == measured2_list and measured1_list == dellist:
                    rospy.logwarn("P1 == P2 == dellist")
                    sum1 = abs(P1[0]) + abs(P1[1]) + abs(P1[2])
                    sum2 = abs(P2[0]) + abs(P2[1]) + abs(P2[2])
                    print "sum1: ", sum1
                    print "sum2: ", sum2      
                    solution = []
                    if sum1 > sum2:
                        solution = P1
                        heading = p1_heading
                        self.psolution = 1
                    else:
                        solution = P2
                        heading = p2_heading
                        self.psolution = 2

                    if bearing > 360-angle_tolerance and heading < 0 + angle_tolerance:
                        bearing = bearing - 360
                    elif heading > 360-angle_tolerance and bearing < 0 + angle_tolerance:
                        bearing = bearing + 360                         

                    angle_diff = abs(heading-bearing)
                    inv_angle_diff = abs(heading - inv_bearing)

                    print "angle_diff: %0.2f" % angle_diff
                    print "inv_angle_diff: %0.2f" % inv_angle_diff

                    if angle_diff < inv_angle_diff and angle_diff < angle_tolerance:
                        x = solution[0]
                        y = solution[1]
                        z = solution[2]
                        rospy.loginfo("vector standard")
                    elif angle_diff > inv_angle_diff and inv_angle_diff < angle_tolerance:
                        x = -solution[0]
                        y = -solution[1]
                        z = -solution[2]
                        rospy.loginfo("vector inverted")
                    else:
                        (x,y) = self.pol2cart(rho,phi)
                        rospy.logerr("defaulting to cardinal")   

                elif measured1_list == dellist:
                    rospy.logwarn("measured1_list")
                    solution = P1
                    heading = p1_heading
                    self.psolution = 1    

                    if bearing > 360-angle_tolerance and heading < 0 + angle_tolerance:
                        bearing = bearing - 360
                    elif heading > 360-angle_tolerance and bearing < 0 + angle_tolerance:
                        bearing = bearing + 360                      

                    angle_diff = abs(heading-bearing)
                    inv_angle_diff = abs(heading - inv_bearing)    

                    print "bearing: %0.2f" % bearing
                    print "inv_bearing: %0.2f" % inv_bearing

                    print "angle_diff: %0.2f" % angle_diff
                    print "inv_angle_diff: %0.2f" % inv_angle_diff

                    if angle_diff < inv_angle_diff and angle_diff < angle_tolerance:
                        x = solution[0]
                        y = solution[1]
                        z = solution[2]
                        rospy.loginfo("vector standard")
                    elif angle_diff > inv_angle_diff and inv_angle_diff < angle_tolerance:
                        x = -solution[0]
                        y = -solution[1]
                        z = -solution[2]
                        rospy.loginfo("vector inverted")
                    else:
                        (x,y) = self.pol2cart(rho,phi)
                        rospy.logerr("defaulting to cardinal")

                elif measured2_list == dellist:
                    rospy.logwarn("measured2_list")
                    solution = P2
                    heading = p2_heading
                    self.psolution = 2  

                    if bearing > 360-angle_tolerance and heading < 0 + angle_tolerance:
                        bearing = bearing - 360
                    elif heading > 360-angle_tolerance and bearing < 0 + angle_tolerance:
                        bearing = bearing + 360                      

                    angle_diff = abs(heading-bearing)
                    inv_angle_diff = abs(heading - inv_bearing) 

                    print "bearing: %0.2f" % bearing
                    print "inv_bearing: %0.2f" % inv_bearing

                    print "angle_diff: %0.2f" % angle_diff
                    print "inv_angle_diff: %0.2f" % inv_angle_diff                          

                    if angle_diff < inv_angle_diff and angle_diff < angle_tolerance:
                        x = solution[0]
                        y = solution[1]
                        z = solution[2]
                        rospy.loginfo("vector standard")
                    elif angle_diff > inv_angle_diff and inv_angle_diff < angle_tolerance:
                        x = -solution[0]
                        y = -solution[1]
                        z = -solution[2]
                        rospy.loginfo("vector inverted")
                    else:
                        (x,y) = self.pol2cart(rho,phi)
                        rospy.logerr("defaulting to cardinal")                    

                elif (measured1_list == measured2_list) and measured1_list != dellist:
                    rospy.logwarn("not equal")
                    sum1 = abs(P1[0]) + abs(P1[1]) + abs(P1[2])
                    sum2 = abs(P2[0]) + abs(P2[1]) + abs(P2[2])
                    print "sum1: ", sum1
                    print "sum2: ", sum2      
                    solution = []
                    if sum1 > sum2:
                        solution = P1
                        heading = p1_heading
                        self.psolution = 1
                    else:
                        solution = P2
                        heading = p2_heading
                        self.psolution = 2

                    if bearing > 360-angle_tolerance and heading < 0 + angle_tolerance:
                        bearing = bearing - 360
                    elif heading > 360-angle_tolerance and bearing < 0 + angle_tolerance:
                        bearing = bearing + 360 

                    angle_diff = abs(heading-bearing)
                    inv_angle_diff = abs(heading - inv_bearing)

                    print "bearing: %0.2f" % bearing
                    print "inv_bearing: %0.2f" % inv_bearing

                    print "angle_diff: %0.2f" % angle_diff
                    print "inv_angle_diff: %0.2f" % inv_angle_diff

                    if angle_diff < inv_angle_diff and angle_diff < angle_tolerance:
                        x = solution[0]
                        y = solution[1]
                        z = solution[2]
                        rospy.loginfo("vector standard")
                    elif angle_diff > inv_angle_diff and inv_angle_diff < angle_tolerance:
                        x = -solution[0]
                        y = -solution[1]
                        z = -solution[2]
                        rospy.loginfo("vector inverted")
                    else:
                        (x,y) = self.pol2cart(rho,phi)
                        rospy.logerr("defaulting to cardinal")                     

                else:
                    (x,y) = self.pol2cart(rho,phi)
                    rospy.logerr("defaulting to cardinal")     
            else:
                (x,y) = self.pol2cart(rho,phi)
                rospy.logerr("defaulting to cardinal")                              

            '''if measured1_list == measured2_list:
                rospy.logwarn("measured1_list == measured2_list")
                p1_min = abs(p1_heading-bearing)
                p2_min = abs(p1_heading-bearing)

                p_min = min(p1_min, p2_min)

                if p_min < 20:
                    if p1_min == p_min:
                        x = P1[0]
                        y = P1[1]
                        z = P1[2]
                        rospy.logwarn("**P1_Crane**")
                    elif p2_min == p_min:
                        x = P2[0]
                        y = P2[1]
                        z = P2[2]
                        rospy.logwarn("**P2_Crane**")
                else:
                    (x,y) = self.pol2cart(rho,phi)
                    rospy.logwarn("**cardinal**")

            elif measured1_list == dellist:# and p1sum > p2sum:
                if abs(p1_heading-bearing) < 20: 
                    x = P1[0]
                    y = P1[1]
                    z = P1[2]
                    rospy.logwarn("**P1_Crane**")
                else:
                    rospy.logwarn("**P1_cardinal**")
                    (x,y) = self.pol2cart(rho,phi)
            elif measured2_list == dellist:# and p2sum > p1sum:
                if abs(p2_heading-bearing) < 20: 
                    x = P2[0]
                    y = P2[1]
                    z = P2[2]
                    rospy.logwarn("**P2_Crane**")
                else:
                    rospy.logwarn("**P2_cardinal**")
                    (x,y) = self.pol2cart(rho,phi)
            else:
                (x,y) = self.pol2cart(rho,phi)'''

        if bearing != -1:  

            self.bearing_pub = rospy.Publisher('hydrophones/bearing_info', Bearing, queue_size = 1)
            self.bearing_pub.publish(Bearing(
                header=Header(stamp=rospy.Time.now(),
                              frame_id='bearing_info'),
                p1 = P1,
                p2 = P2,
                cardinal_bearing = bearing, 
                dels = self.sorted_dels,
                psolution = self.psolution
                ))

            self.crane_pub = rospy.Publisher('hydrophones/crane_pos', Crane_pos, queue_size = 1)
            self.crane_pub.publish(Crane_pos(
                header=Header(stamp=rospy.Time.now(),
                              frame_id='Crane_pos_calc'),
                x_pos=x,
                y_pos=y,
                z_pos=z))          

        return Crane_pos_serviceResponse(x, y, z)        

    def __init__(self):
        rospy.init_node('crane_method_service')
        #rospy.Subscriber('/hydrophones/actual_time_stamps', Actual_time_stamps, self.calc_vals) #self.actu_vals)
        rospy.Subscriber('/hydrophones/calculated_time_stamps', Calculated_time_stamps, self.calc_vals)
        #rospy.Subscriber('hydrophones/hydrophone_locations', Hydrophone_locations, self.hydrophone_locations)

        #self.crane_serv = rospy.Service('hydrophones/crane_srv', Crane_pos_service, self.crane_solver)
        self.cardinal_pub = rospy.Publisher('hydrophones/cardinal', Float32, queue_size = 1)

        self.ref_hydro = 0
        self.psolution = 0

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


'''
               elif self.psolution == 1 and abs(del1)+abs(del2)+abs(del3) > 100.0:
                    print "p1_head - bearing %0.2f" % abs(p1_heading -bearing)
                    if abs(p1_heading-bearing) < 30:
                            x = P1[0]
                            y = P1[1]
                            z = P1[2]
                            rospy.logerr("P1 normal")
                    else:
                        rospy.logerr(p1_heading)
                        if p1_heading > 180:
                            p1_heading = p1_heading - 180
                        else:
                            p1_heading = 180 + p1_heading
                        
                        if p1_heading > 360:
                            p1_heading = p1_heading - 360
                        print "p1_heading else: %0.2f" % p1_heading                        
                        if abs(p1_heading-bearing) < 30 or abs((360-p1_heading)-bearing) < 30:
                            x = -P1[0]
                            y = -P1[1]
                            z = P1[2]
                            rospy.logerr("P1 sign flip")
                        else:
                            (x,y) = self.pol2cart(rho,phi)
                            rospy.logerr("defaulting to cardinal")                                                 
                elif self.psolution == 2 and abs(del1)+abs(del2)+abs(del3) > 100.0:
                    print "p2_head - bearing %0.2f" % abs(p2_heading -bearing)
                    if abs(p2_heading-bearing) < 30:
                            x = P2[0]
                            y = P2[1]
                            z = P2[2]
                            rospy.logerr("P2 normal")
                    else:
                        p2_heading = 180 + p2_heading


                        if p2_heading > 360:
                            p2_heading = p2_heading - 360
                        print "p2_heading else: %0.2f" % p2_heading
                        if abs(p2_heading-bearing) < 30 or abs((360-p2_heading)-bearing) < 30:
                            x = -P2[0]
                            y = -P2[1]
                            z = P2[2]    
                            rospy.logerr("P2 sign flip") 
                        else:
                            (x,y) = self.pol2cart(rho,phi)
                            rospy.logerr("defaulting to cardinal")        '''