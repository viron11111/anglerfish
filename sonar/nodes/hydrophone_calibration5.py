#!/usr/bin/env python
import rospy
import rosparam
import random
import numpy as np
import matplotlib.pyplot as plt
import operator
import os

from std_msgs.msg import Header, Float32
from pinger_tracker.msg import *
from sonar.msg import Bearing
#from multilateration import Multilaterator, ReceiverArraySim, Pulse

import sys
import math

from dynamic_reconfigure.server import Server
from pinger_tracker.cfg import SignalConfig
from pinger_tracker.srv import *

import time

#1600, -1600, -2000 (mm)
#0.0, 9.119770766119473, -9.016221156343818, -9.016221156343818


class solver():

    def run_calibration(self):

        c = 1.484 # speed of sound in 20 C water per uSec
        #c = 0.343 #speed of sound in air

        #starting values
        x1_orig = 167.7 #in mm
        x2_orig = 82.1
        y2_orig = -178.5
        x3_orig = 46.2
        y3_orig = -24.0
        z3_orig = -89.5

        mm_range = .1 #mm 5 = +- 2.5
        mm_resolution = .01 #mm

        number_of_iterations = mm_range/mm_resolution

        lowest_error = 12.26 #(11.152 or 16.64% error)

        #experimentally gathered
        tstamps = [[0.00, 146.0, 88.0, 91.5], [10.0, 143.0, 70.75, 84.5], [20.0, 138.0, 52.0, 76.75], [30.0, 130.5, 33.75, 67.75],
                    [40.0, 121.0,  15.0,  59.25], [50.0, 108.0, -4.75, 49.25], [60.0, 91.0, -24.0, 36.75],[70.0, 74.0, -71.5, 25.5], 
                    [80.0, 55.0, -87.0, 14.5], [90.0, 34.0, -98.0, 3.5], [110.0, -4.5, -112.0, -13.5],
                    [130.0, -42.0, -81.0, -26.0], [140.0, -55.5, -75.5, -29.0], [150.0, -65.5, -67.0, -30.5],
                    [160.0, -73.5, -57.0, -30.25], [170.0, -111.5,  -43.5, -28.25], [180.0, -115.5, -29.5, -24.75], [190.0, -115.0, -11.5, -19.0],
                    [200.0, -110.0, 8.5, -11.0], [220.0, -122.0, 14.5, 6.5], [230.0, -72.75, 39.5, 21],
                    [240.0, -57.5, 56.75, 32.0], [250.0, -41.25, 72.5, 43.5],[260.0, -18.0, 87.25, 55.0], [270.0, 1.0, 97.5, 64.5],
                    [280.0, 18.5, 105.75, 74.0], [290.0, 39.75, 113.25, 85], [300.0, 57.0, 114.5, 91.25], [310.0, 73.0, 111.5, 94.5],
                    [330.0, 96.5, 98.0, 98.0], [340.0, 107.0, 88.0, 98.75],[350.0, 112.5, 75.75, 96.0]]

        for i in range(int(number_of_iterations)+1):
            x1 = x1_orig - (mm_range/2.0) + (mm_resolution*i)
            print "******************************"
            print "***** Iteration %i of %i *****" % (i, number_of_iterations)
            print "******************************"
            for j in range(int(number_of_iterations)+1):
                x2 = x2_orig - (mm_range/2.0) + (mm_resolution*j)
                for k in range(int(number_of_iterations)+1):
                    y2 = y2_orig - (mm_range/2.0) + (mm_resolution*k)
                    for l in range(int(number_of_iterations)+1):
                        x3 = x3_orig - (mm_range/2.0) + (mm_resolution*l)
                        for m in range(int(number_of_iterations)+1):
                            y3 = y3_orig - (mm_range/2.0) + (mm_resolution*m)
                            for n in range(int(number_of_iterations)+1):
                                z3 = z3_orig - (mm_range/2.0) + (mm_resolution*n)
                                
                                error_sum = 0
                                for o in range(len(tstamps)):
                                        
                                    hydro1 = [x1,      0,     0]
                                    hydro2 = [x2,      y2,     0]
                                    hydro3 = [x3,      y3,     z3]
                                    
                                    self.bearing = self.cardinal(tstamps[o][1],tstamps[o][2],tstamps[o][3])
                                    
                                    del1 = (tstamps[o][1])*c #mm/uSec
                                    del2 = (tstamps[o][2])*c #mm/uSec
                                    del3 = (tstamps[o][3])*c #mm/uSec

                                    output = self.crane_calc(del1, del2, del3, hydro1, hydro2, hydro3)

                                    if output < 90 and tstamps[o][0] > 270:
                                        #print "turn over error!!!!!!!!"
                                        output = output + 360
                                    elif output > 270 and tstamps[o][0] < 90:
                                        output = tstamps[o][0] + 360

                                    #print "output: %f actual: %f error: %f" % (output, tstamps[o][0], abs(output-tstamps[o][0]))                                        

                                    error_sum = error_sum + abs(output-tstamps[o][0])                                

                                error = float(error_sum/len(tstamps))
                                #print error

                                if error < lowest_error:
                                    lowest_error = error                                     
                                    print "lowest_error: %0.2f" % lowest_error
                                    print "hydro1: [%s] hydro2: [%s] hydro3: [%s]" % (', '.join(map(str, hydro1)),', '.join(map(str, hydro2)),', '.join(map(str, hydro3))) 

                                #intsum = int(error_sum)
                                #if intsum != 89:
                                #   print intsum
                                

    def crane_solutions(self, data):
        rospy.loginfo("service response crane_srv")
        x = 3000
        y = 1000
        z = 2000
        return x,y,z

    def cardinal(self, del1, del2, del3):
        del0 = 0.0
        self.bearing = 0.0
        tolerance = 15
        self.psolution = 0
        dels = {"del0": del0, "del1": del1, "del2": del2, "del3": del3}
        sorted_dels = sorted(dels.items(), key=operator.itemgetter(1))
        sorted_dels = (sorted_dels[0][0],sorted_dels[1][0],sorted_dels[2][0],sorted_dels[3][0])  
        self.sorted_dels = sorted_dels   
        #print self.sorted_dels
        if sorted_dels == ('del2', 'del3', 'del0', 'del1'): #double checked
            if abs(del1-del0) < tolerance:
                self.bearing = 90.0
            elif abs(del0-del3) < tolerance:
                self.bearing = 60.0
            else:
                self.bearing = 75.0  
            self.psolution = 1
        elif sorted_dels == ('del3', 'del2', 'del0', 'del1'): #new
            if abs(del1-del0) < tolerance:
                self.bearing = 90.0
            elif abs(del0-del3) < tolerance:
                self.bearing = 60.0
            else:
                self.bearing = 75.0  
            self.psolution = 1            
        elif sorted_dels == ('del2', 'del3', 'del1', 'del0'): #double checked
            if abs(del1-del0) < tolerance:
                self.bearing = 90.0
            elif abs(del1-del3) < tolerance:
                self.bearing = 120
            else:
                self.bearing = 105     
            self.psolution = 1  #changed changed back
        elif sorted_dels == ('del3', 'del2', 'del1', 'del0'): #new
            if abs(del1-del2) < tolerance:
                self.bearing = 150         
            elif abs(del1-del3) < tolerance:
                self.bearing = 120
            else:
                self.bearing = 135
            self.psolution = 1
        elif sorted_dels == ('del2', 'del1', 'del3', 'del0'): #double checked
            if abs(del1-del2) < tolerance:
                self.bearing = 150         
            elif abs(del1-del3) < tolerance:
                self.bearing = 120
            else:
                self.bearing = 135
            self.psolution = 1
        elif sorted_dels == ('del1', 'del2', 'del3', 'del0'): #double checked
            if abs(del3-del2) < tolerance:
                self.bearing = 180
            elif abs(del1-del2) < tolerance:
                self.bearing = 150
            else:                
                self.bearing = 165.0
            self.psolution = 1
        elif sorted_dels == ('del1', 'del3', 'del2', 'del0'): #switching between 1 and 2
            if abs(del2-del3) < tolerance:
                self.bearing = 180
            elif abs(del2-del0) < tolerance:
                self.bearing = 210
            else:                            
                self.bearing = 195
            self.psolution = 1
        elif sorted_dels == ('del3', 'del1', 'del2', 'del0'): #new
            if abs(del2-del3) < tolerance:
                self.bearing = 180
            elif abs(del2-del0) < tolerance:
                self.bearing = 210
            else:                            
                self.bearing = 195
            self.psolution = 1            
            #print "here2" 
        elif sorted_dels == ('del1', 'del3', 'del0', 'del2'): #double checked
            if abs(del2) < tolerance:
                self.bearing = 210
            elif abs(del3-del0)<tolerance:
                self.bearing = 240
            else:
                self.bearing = 225
            self.psolution = 2
        elif sorted_dels == ('del3', 'del1', 'del0', 'del2'): #new
            if abs(del1-del0) < tolerance:
                self.bearing = 270.0
            elif abs(del0-del3) < tolerance:
                self.bearing = 240.0
            else:
                self.bearing = 255.0
            self.psolution = 1
        elif sorted_dels == ('del1', 'del0', 'del3', 'del2'): #double checked
            if abs(del1-del0) < tolerance:
                self.bearing = 270.0
            elif abs(del0-del3) < tolerance:
                self.bearing = 240.0
            else:
                self.bearing = 255.0
            self.psolution = 1
        elif sorted_dels == ('del0', 'del1', 'del3', 'del2'):#double checked
            if abs(del1-del0) < tolerance:
                self.bearing = 270.0
            elif abs(del3-del1) < tolerance:
                self.bearing = 300.0
            else:
                self.bearing = 285.0
            self.psolution = 2
            #print "here4" 
        elif sorted_dels == ('del0', 'del3', 'del1', 'del2'):#double checked
            if abs(del1-del2) < tolerance:
                self.bearing = 330
            elif abs(del1-del3) < tolerance:
                self.bearing = 300
            else:                            
                self.bearing = 315 
            self.psolution = 1
        elif sorted_dels == ('del3', 'del0', 'del1', 'del2'):#new
            if abs(del1-del2) < tolerance:
                self.bearing = 330
            elif abs(del1-del3) < tolerance:
                self.bearing = 300
            else:                            
                self.bearing = 315 
            self.psolution = 1            
        elif sorted_dels == ('del0', 'del3', 'del2', 'del1'):#double checked 
            if abs(del1-del2) < tolerance:
                self.bearing = 330.0
            elif abs(del2-del3) < tolerance:
                self.bearing = 0.0
            else:                            
                self.bearing = 345.0
            self.psolution = 2
        elif sorted_dels == ('del0', 'del2', 'del1', 'del3'):#new bottom
            if abs(del1-del2) < tolerance:
                self.bearing = 330.0
            elif abs(del2-del3) < tolerance:
                self.bearing = 0.0
            else:                            
                self.bearing = 345.0
            self.psolution = 2            
        elif sorted_dels == ('del0', 'del1', 'del2', 'del3'):#new bottom
            if abs(del1-del2) < tolerance:
                self.bearing = 330
            elif abs(del1-del3) < tolerance:
                self.bearing = 300
            else:                            
                self.bearing = 315 
            self.psolution = 1 
        elif sorted_dels == ('del1', 'del0', 'del2', 'del3'):#new bottom
            if abs(del1-del0) < tolerance:
                self.bearing = 270.0
            elif abs(del0-del3) < tolerance:
                self.bearing = 240.0
            else:
                self.bearing = 255.0
            self.psolution = 1
        elif sorted_dels == ('del1', 'del2', 'del0', 'del3'):#new bottom
            if abs(del2-del3) < tolerance:
                self.bearing = 180
            elif abs(del2-del0) < tolerance:
                self.bearing = 210
            else:                            
                self.bearing = 195
            self.psolution = 1    
        elif sorted_dels == ('del2', 'del1', 'del0', 'del3'):#new bottom 
            if abs(del3-del2) < tolerance:
                self.bearing = 180
            elif abs(del1-del2) < tolerance:
                self.bearing = 150
            else:                
                self.bearing = 165.0
            self.psolution = 1
        elif sorted_dels == ('del2', 'del0', 'del1', 'del3'):#new bottom             
            if abs(del1-del0) < tolerance:
                self.bearing = 90.0
            elif abs(del0-del3) < tolerance:
                self.bearing = 60.0
            else:
                self.bearing = 75.0  
            self.psolution = 1  

            
        elif sorted_dels == ('del0', 'del2', 'del3', 'del1'):#double checked             
            if abs(del2-del3) < tolerance:            
                self.bearing = 0.0
            elif abs(del2-del0) < tolerance:
                self.bearing = 30.0
            else:
                self.bearing = 15.0  
            self.psolution = 2  
        elif sorted_dels == ('del2', 'del0', 'del3', 'del1'):#double checked 
            if abs(del0-del3) < tolerance:
                self.bearing = 60.0
            if abs(del0-del2) < tolerance:
                self.bearing = 30.0
            else:
                self.bearing = 45.0
            self.psolution = 2
            #print "here6" 
        elif sorted_dels == ('del3', 'del0', 'del2', 'del1'): #new
            if abs(del2-del3) < tolerance:            
                self.bearing = 0.0
            elif abs(del2-del0) < tolerance:
                self.bearing = 30.0
            else:
                self.bearing = 15.0  
            self.psolution = 2           

        else:
            rospy.logerr("CARDINAL failed to find solution!")
            os.system("rosnode kill crane_method_service")
            self.bearing = -1

        if sorted_dels[3] == 'del0':
            self.ref_hydro = 0
        elif sorted_dels[3] == 'del1':
            self.ref_hydro = 1
        elif sorted_dels[3] == 'del2':
            self.ref_hydro = 2       
        return self.bearing
        
        #print sorted_dels
    def pol2cart(self, rho, phi):
        x = rho * np.cos(phi)
        y = rho * np.sin(phi)
        return(x, y)        

    def crane_calc(self,del1i,del2i,del3i,hydro1,hydro2,hydro3):

        #print "del1: %0.10f del2: %0.10f del3: %0.10f" % (del1i, del2i, del3i)        
       
        del1 = del1i
        del2 = del2i
        del3 = del3i 

        x1 = hydro1[0] #+ 0.75 #np.random.uniform(-1, 1)  #in mm
        x2 = hydro2[0] #- 0.75 #np.random.uniform(-1, 1)
        y2 = hydro2[1] #+ 0.75 #np.random.uniform(-1, 1)
        x3 = hydro3[0] #- 0.75 #np.random.uniform(-1, 1)
        y3 = hydro3[1] #+ 0.75 #np.random.uniform(-1, 1)
        z3 = hydro3[2]            

        P1 = [0,0,0]
        P2 = [0,0,0]        

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

        phi = math.radians(360-self.bearing)
        rho = 500.0


        if (discr < 0):
            #rospy.logerr("no real solution was found; set garbage values for P1 and P2")

            #print discr

            (x,y) = self.pol2cart(rho,phi)

            P1[0] = P1[1] = P1[2] = 0.0
            P2[0] = P2[1] = P2[2] = 0.0
            z=0         

        else:

            #print "worked"

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

            #rospy.loginfo("x1: %f" % (P1[0]))
            #rospy.loginfo("y1: %f" % (P1[1]))
            #rospy.loginfo("z1: %f" % (P1[2]))
            #rospy.loginfo("**")
            #rospy.loginfo("x2: %f" % (P2[0]))
            #rospy.loginfo("y2: %f" % (P2[1]))
            #rospy.loginfo("z2: %f" % (P2[2]))

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

            #print "dellist:        ", dellist
            #print "measured1_list: ", measured1_list
            #print "measured2_list: ", measured2_list

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

            #rospy.loginfo("p1_heading: %0.2f" % p1_heading)
            #rospy.loginfo("p2_heading: %0.2f" % p2_heading)

            inv_bearing = self.bearing + 180   
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

            angle_tolerance = 89

            if map(abs,measured1_list) == map(abs,dellist) or map(abs,measured2_list) == map(abs,dellist):
                sum1 = abs(P1[0]) + abs(P1[1]) + abs(P1[2])
                sum2 = abs(P2[0]) + abs(P2[1]) + abs(P2[2])

                if measured1_list == measured2_list and measured1_list == dellist:
                    #rospy.logwarn("P1 == P2 == dellist")
                    sum1 = abs(P1[0]) + abs(P1[1]) + abs(P1[2])
                    sum2 = abs(P2[0]) + abs(P2[1]) + abs(P2[2])
                    #print "sum1: ", sum1
                    #print "sum2: ", sum2      
                    solution = []
                    if sum1 > sum2:
                        solution = P1
                        heading = p1_heading
                        self.psolution = 1
                        #print "P1"
                    else:
                        solution = P2
                        heading = p2_heading
                        self.psolution = 2
                        #print "P2"

                    if self.bearing > 360-angle_tolerance and heading < 0 + angle_tolerance:
                        self.bearing = self.bearing - 360
                    elif heading > 360-angle_tolerance and self.bearing < 0 + angle_tolerance:
                        self.bearing = self.bearing + 360                         

                    angle_diff = abs(heading-self.bearing)
                    inv_angle_diff = abs(heading - inv_bearing)

                    #print "angle_diff: %0.2f" % angle_diff
                    #print "inv_angle_diff: %0.2f" % inv_angle_diff

                    if angle_diff < inv_angle_diff and angle_diff < angle_tolerance:
                        x = solution[0]
                        y = solution[1]
                        z = solution[2]
                        #rospy.loginfo("vector standard")
                    elif angle_diff > inv_angle_diff and inv_angle_diff < angle_tolerance:
                        x = -solution[0]
                        y = -solution[1]
                        z = -solution[2]
                        #rospy.loginfo("vector inverted")
                    else:
                        (x,y) = self.pol2cart(rho,phi)
                        #rospy.logerr("defaulting to cardinal")   

                elif sum1 > sum2:
                    #rospy.logwarn("measured1_list")
                    solution = P1
                    heading = p1_heading
                    self.psolution = 1    
                    #print "P1"

                    if self.bearing > 360-angle_tolerance and heading < 0 + angle_tolerance:
                        self.bearing = self.bearing - 360
                    elif heading > 360-angle_tolerance and self.bearing < 0 + angle_tolerance:
                        self.bearing = self.bearing + 360                      

                    angle_diff = abs(heading-self.bearing)
                    inv_angle_diff = abs(heading - inv_bearing)    

                    #print "bearing: %0.2f" % self.bearing
                    #print "inv_bearing: %0.2f" % inv_bearing

                    #print "angle_diff: %0.2f" % angle_diff
                    #print "inv_angle_diff: %0.2f" % inv_angle_diff

                    if angle_diff < inv_angle_diff and angle_diff < angle_tolerance:
                        x = solution[0]
                        y = solution[1]
                        z = solution[2]
                        #rospy.loginfo("vector standard")
                    elif angle_diff > inv_angle_diff and inv_angle_diff < angle_tolerance:
                        x = -solution[0]
                        y = -solution[1]
                        z = -solution[2]
                        #rospy.loginfo("vector inverted")
                    else:
                        (x,y) = self.pol2cart(rho,phi)
                        #rospy.logerr("defaulting to cardinal")

                elif sum2 > sum1:
                    #rospy.logwarn("measured2_list")
                    solution = P2
                    heading = p2_heading
                    self.psolution = 2  
                    #print "P2"

                    if self.bearing > 360-angle_tolerance and heading < 0 + angle_tolerance:
                        self.bearing = self.bearing - 360
                    elif heading > 360-angle_tolerance and self.bearing < 0 + angle_tolerance:
                        self.bearing = self.bearing + 360                      

                    angle_diff = abs(heading-self.bearing)
                    inv_angle_diff = abs(heading - inv_bearing) 

                    #print "bearing: %0.2f" % self.bearing
                    #print "inv_bearing: %0.2f" % inv_bearing

                    #print "angle_diff: %0.2f" % angle_diff
                    #print "inv_angle_diff: %0.2f" % inv_angle_diff                          

                    if angle_diff < inv_angle_diff and angle_diff < angle_tolerance:
                        x = solution[0]
                        y = solution[1]
                        z = solution[2]
                        #rospy.loginfo("vector standard")
                    elif angle_diff > inv_angle_diff and inv_angle_diff < angle_tolerance:
                        x = -solution[0]
                        y = -solution[1]
                        z = -solution[2]
                        #rospy.loginfo("vector inverted")
                    else:
                        (x,y) = self.pol2cart(rho,phi)
                        #rospy.logerr("defaulting to cardinal")                    

                elif (measured1_list == measured2_list) and measured1_list != dellist:
                    #rospy.logwarn("not equal")
                    sum1 = abs(P1[0]) + abs(P1[1]) + abs(P1[2])
                    sum2 = abs(P2[0]) + abs(P2[1]) + abs(P2[2])
                    #print "sum1: ", sum1
                    #print "sum2: ", sum2      
                    solution = []
                    if sum1 > sum2:
                        solution = P1
                        heading = p1_heading
                        self.psolution = 1
                        #print "P1"
                    else:
                        solution = P2
                        heading = p2_heading
                        self.psolution = 2
                        #print "P2"

                    if self.bearing > 360-angle_tolerance and heading < 0 + angle_tolerance:
                        self.bearing = self.bearing - 360
                    elif heading > 360-angle_tolerance and self.bearing < 0 + angle_tolerance:
                        self.bearing = self.bearing + 360 

                    angle_diff = abs(heading-self.bearing)
                    inv_angle_diff = abs(heading - inv_bearing)

                    #print "bearing: %0.2f" % self.bearing
                    #print "inv_bearing: %0.2f" % inv_bearing

                    #print "angle_diff: %0.2f" % angle_diff
                    #print "inv_angle_diff: %0.2f" % inv_angle_diff

                    if angle_diff < inv_angle_diff and angle_diff < angle_tolerance:
                        x = solution[0]
                        y = solution[1]
                        z = solution[2]
                        #rospy.loginfo("vector standard")
                    elif angle_diff > inv_angle_diff and inv_angle_diff < angle_tolerance:
                        x = -solution[0]
                        y = -solution[1]
                        z = -solution[2]
                        #rospy.loginfo("vector inverted")
                    else:
                        (x,y) = self.pol2cart(rho,phi)
                        #rospy.logerr("defaulting to cardinal")                     

                else:
                    (x,y) = self.pol2cart(rho,phi)
                    #rospy.logerr("defaulting to cardinal")     
            else:
                (x,y) = self.pol2cart(rho,phi)
                #rospy.logerr("defaulting to cardinal")   

        return_heading = np.arctan2(x,y) - np.pi/2
        return_heading = math.degrees(return_heading)
        #print return_heading

        if return_heading < 0:
            return_heading  = 360 + return_heading


        return (return_heading)     

    def calc_vals(self, data):

        c = 1.484 # speed of sound in 20 C water per uSec
        #c = 0.343 #speed of sound in air 

        self.bearing = self.cardinal(data.calculated_time_stamps[1],data.calculated_time_stamps[2],data.calculated_time_stamps[3])        

        P1 = [0,0,0]
        P2 = [0,0,0]

        del1i = (data.calculated_time_stamps[1])*c #mm/uSec
        del2i = (data.calculated_time_stamps[2])*c #mm/uSec
        del3i = (data.calculated_time_stamps[3])*c #mm/uSec

        print "*******************POSITION 1*******************"
        print "del1: %0.10f del2: %0.10f del3: %0.10f" % (del1i, del2i, del3i)         

        hydro0_xyz = [0,      0,     0]
        hydro1_xyz = [-173.2,   0,     0]
        hydro2_xyz = [-86.6,  -150.0,     0]
        hydro3_xyz = [-86.6,  -50.0, -100.0]

        (x1,y1,z1,P11,P21) = self.crane_calc(del1i,del2i,del3i,hydro1_xyz,hydro2_xyz,hydro3_xyz)    

        x = x1 #-173.2-173.2 #(x1+x2)/2
        y = y1 #(y1+y2)/2
        z = z1 #(z1+z2)/2       

        if self.bearing != -1:  

            self.bearing_pub = rospy.Publisher('hydrophones/bearing_info', Bearing, queue_size = 1)
            self.bearing_pub.publish(Bearing(
                header=Header(stamp=rospy.Time.now(),
                              frame_id='bearing_info'),
                p1 = P11,
                p2 = P21,
                cardinal_bearing = self.bearing, 
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

        (x,y,z) = (x1,y1,z1)         

        return Crane_pos_serviceResponse(x, y, z)        

    def __init__(self):

        self.run_calibration()


def main():
    
    solver()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print "Shutting down"
        pass
    


if __name__ == '__main__':
    main()              