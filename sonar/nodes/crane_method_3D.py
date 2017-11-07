#!/usr/bin/env python
import rospy
import rosparam
import random
import numpy as np
import matplotlib.pyplot as plt
import operator

from std_msgs.msg import Header, Float32
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

    def cardinal(self, del1, del2, del3):
        del0 = 0.0
        bearing = 0.0
        dels = {"del0": del0, "del1": del1, "del2": del2, "del3": del3}
        sorted_dels = sorted(dels.items(), key=operator.itemgetter(1))
        sorted_dels = (sorted_dels[0][0],sorted_dels[1][0],sorted_dels[2][0],sorted_dels[3][0])        
        if sorted_dels == ('del2', 'del3', 'del0', 'del1'):
            if del1 > -3.0 and del1 < 3.0:
                bearing = 90.0
            else:
                bearing = 67.5  
        elif sorted_dels == ('del2', 'del3', 'del1', 'del0'):                        
            if del1 > -3.0 and del1 < 3.0:
                bearing = 90.0
            else:
                bearing = 112.5              
        elif sorted_dels == ('del2', 'del1', 'del3', 'del0'):
            if abs(del1-del2) < 6.0:
                bearing = 135            
            else:
                bearing = 112.5
        elif sorted_dels == ('del1', 'del2', 'del3', 'del0'):          
            if abs(del3-del2) < 6.0:
                bearing = 180
            else:                
                bearing = 157.5
        elif sorted_dels == ('del1', 'del3', 'del2', 'del0'):
            if abs(del2-del3) < 6.0:
                bearing = 180
            else:                            
                bearing = 202.5
        elif sorted_dels == ('del1', 'del3', 'del0', 'del2'):
            if abs(del2) < 6.0:
                bearing = 225.0
            else:
                bearing = 247.5
        elif sorted_dels == ('del1', 'del0', 'del3', 'del2'):
            if del1 > -3.0 and del1 < 3.0:
                bearing = 270.0
            else:
                bearing = 247.5
        elif sorted_dels == ('del0', 'del1', 'del3', 'del2'):
            if del1 > -3.0 and del1 < 3.0:
                bearing = 270.0
            else:
                bearing = 292.5            
        elif sorted_dels == ('del0', 'del3', 'del1', 'del2'):
            if abs(del1-del2) < 6.0:
                bearing = 315.0
            else:                            
                bearing = 292.5            
        elif sorted_dels == ('del0', 'del3', 'del2', 'del1'):        
            if abs(del1-del2) < 6.0:
                bearing = 315.0
            elif abs(del2-del3) < 6.0:
                bearing = 0.0
            else:                            
                bearing = 337.5
        elif sorted_dels == ('del0', 'del2', 'del3', 'del1'):            
            if abs(del2-del3) < 6.0:            
                bearing = 0.0
            else:
                bearing = 22.5
        elif sorted_dels == ('del2', 'del0', 'del3', 'del1'):
            bearing = 45.0

        self.cardinal_pub = rospy.Publisher('hydrophones/cardinal', Float32, queue_size = 1)
        self.cardinal_pub.publish(Float32(bearing))


        rospy.logerr(bearing)       
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
        #hydro0_xyz = [0,      0,     0]
        #hydro1_xyz = [100.0,   0,     0]
        #hydro2_xyz = [-50,  86.6,     0]
        #hydro3_xyz = [-50,  -86.6, 0]

        

        #experimental layout
        hydro0_xyz = [0,      0,     0]
        hydro1_xyz = [-173.2,   0,     0]
        hydro2_xyz = [-86.6,  -150,     0]
        hydro3_xyz = [-86.6,  -50, -100]     

        #experimental layout
        hydro0_xyz = [0,      0,     0]
        hydro1_xyz = [-168,   0,     0]
        hydro2_xyz = [-86.6,  -149,     0]
        hydro3_xyz = [-86.6,  -49, -100]        

      

        x1 = hydro1_xyz[0] #+ 0.75 #np.random.uniform(-1, 1)  #in mm
        x2 = hydro2_xyz[0] #- 0.75 #np.random.uniform(-1, 1)
        y2 = hydro2_xyz[1] #+ 0.75 #np.random.uniform(-1, 1)
        x3 = hydro3_xyz[0] #- 0.75 #np.random.uniform(-1, 1)
        y3 = hydro3_xyz[1] #+ 0.75 #np.random.uniform(-1, 1)
        z3 = hydro3_xyz[2]

        bearing = self.cardinal(data.calculated_time_stamps[1],data.calculated_time_stamps[2],data.calculated_time_stamps[3])

        del1 = (data.calculated_time_stamps[1])*c #mm/uSec
        del2 = (data.calculated_time_stamps[2])*c #mm/uSec
        del3 = (data.calculated_time_stamps[3])*c #mm/uSec   

        '''print "x1: ", x1
        print "x2: ", x2
        print "y2: ", y2
        print "x3: ", x3
        print "y3: ", y3
        print "z3: ", z3
        print "del1: ", del1
        print "del2: ", del2
        print "del3: ", del3'''

        #Values from Dr. Crane's documentation:
        '''x1 = 8.5
        x2 = 5.65
        y2 = 4.75
        x3 = -1.75
        y3 = 3.5
        z3 = 1.75        

        del1 = 2.053896
        del2 = -1.349004
        del3 = -2.817121'''

        print "********"

        #print "\n"
        #print "del1: %f del2: %f del3: %f" % (del1, del2, del3)        


        left = x1/del1 if del1 != 0 else 0
        right = x2/del2 if del2 != 0 else 0

        #print "left = %f B1 = %f right = %f" % (left, B1, right)

        A1 = left - right    # eqn (12)
        B1 = -y2/del2 if del2 != 0 else 0

        #print "A1: %f" % A1
        
        catch = (x1*x1-del1*del1)/(2.0*del1) if del1 != 0 else 0
        zero = (x2*x2 + y2*y2-del2*del2)/(2.0*del2) if del2 != 0 else 0

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

        '''if del1 > del3:
            rospy.logwarn("del1 > del3")

            P1[0] = (-BB+math.sqrt(discr))/(2.0*AA) if (2.0*AA) != 0 else 0
            P2[0] = (-BB-math.sqrt(discr))/(2.0*AA) if (2.0*AA) != 0 else 0

            # get corresponding value for y coordinate  ;  eqn (11)
            P1[1] = -(A1*P1[0]+D1)/B1 if B1 != 0 else 0
            P2[1] = -(A1*P2[0]+D1)/B1 if B1 != 0 else 0

            # get correspoinding value for z coordinate ;  eqn (16)
            P1[2] = -(Q2a*P1[0]+Q2b)/Q1 if Q1 != 0 else 0
            P2[2] = -(Q2a*P2[0]+Q2b)/Q1 if Q1 != 0 else 0

            P1sum = abs(P1[0]) + abs(P1[1]) + abs(P1[2])
            P2sum = abs(P2[0]) + abs(P2[1]) + abs(P2[2])

            rospy.loginfo("x1: %f" % (P1[0]))
            rospy.loginfo("y1: %f" % (P1[1]))
            rospy.loginfo("z1: %f" % (P1[2]))

            rospy.loginfo("x2: %f" % (P2[0]))
            rospy.loginfo("y2: %f" % (P2[1]))
            rospy.loginfo("z2: %f" % (P2[2]))'''

        phi = math.radians(360-bearing)
        rho = 500.0


        if (discr < 0):
            rospy.loginfo("no real solution was found; set garbage values for P1 and P2")

            (x,y) = self.pol2cart(rho,phi)

            P1[0] = P1[1] = P1[2] = 0.0
            P2[0] = P2[1] = P2[2] = 0.0
            z=0

        else:

            #print "BB: ", BB
            #print "discr: ", discr
            #print "AA: ", AA

            P1[0] = (-BB+math.sqrt(discr))/(2.0*AA) if (2.0*AA) != 0 else 0
            P2[0] = (-BB-math.sqrt(discr))/(2.0*AA) if (2.0*AA) != 0 else 0

            # get corresponding value for y coordinate  ;  eqn (11)
            P1[1] = -(A1*P1[0]+D1)/B1 if B1 != 0 else 0
            P2[1] = -(A1*P2[0]+D1)/B1 if B1 != 0 else 0

            # get correspoinding value for z coordinate ;  eqn (16)
            P1[2] = -(Q2a*P1[0]+Q2b)/Q1 if Q1 != 0 else 0
            P2[2] = -(Q2a*P2[0]+Q2b)/Q1 if Q1 != 0 else 0

            P1sum = abs(P1[0]) + abs(P1[1]) + abs(P1[2])
            P2sum = abs(P2[0]) + abs(P2[1]) + abs(P2[2])

            print "P1sum: %d" % P1sum
            rospy.loginfo("x1: %f" % (P1[0]))
            rospy.loginfo("y1: %f" % (P1[1]))
            rospy.loginfo("z1: %f" % (P1[2]))

            print "P2sum: %d" % P2sum
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

            #print measured2_list == dellist

            print "dellist:        ", dellist
            print "measured1_list: ", measured1_list
            print "measured2_list: ", measured2_list

            x=0
            y=0
            z=0
            p1sum = abs(P1[0])+abs(P1[1])+abs(P1[2])
            p2sum = abs(P2[0])+abs(P2[1])+abs(P2[2])            

            if del1 > del3:
                rospy.logwarn("del1 > del3")
                if measured1_list == measured2_list:
                    if p1sum > p2sum:
                        x = P1[0]
                        y = P1[1]
                        z = P1[2]
                        rospy.logwarn("P1SUM")
                    else:
                        x = P2[0]
                        y = P2[1]
                        z = P2[2]
                        rospy.logwarn("P2SUM")
                elif (dellist[0] > 0 and dellist[1] > 0 and dellist[2] > 0) and p1sum > p2sum and P1[0] < 0 and P1[1] <0:
                        x = -P1[0]
                        y = -P1[1]
                        z = P1[2]
                        rospy.logwarn("P1SUM Wierd case")
                elif measured1_list == dellist:# and p1sum > p2sum:
                    x = P1[0]
                    y = P1[1]
                    z = P1[2]
                    rospy.logwarn("**P1**")
                elif measured2_list == dellist:# and p2sum > p1sum:
                    x = P2[0]
                    y = P2[1]                
                    z = P2[2]
                    rospy.logwarn("**P2**")
                '''elif p1sum > p2sum:
                    x = P1[0]
                    y = P1[1]
                    z = P1[2]
                    rospy.logwarn("P1SUM last")
                else:
                    x = P2[0]
                    y = P2[1]
                    z = P2[2]
                    rospy.logwarn("P2SUM last")'''
            elif measured1_list == dellist:
                x = P1[0]
                y = P1[1]
                z = P1[2]
                print "**P1**"
                #rospy.loginfo("x1: %f" % (P1[0]))
                #rospy.loginfo("y1: %f" % (P1[1]))
                #rospy.loginfo("z1: %f" % (P1[2]))
            elif measured2_list == dellist:
                x = P2[0]
                y = P2[1]                
                z = P2[2]
                print "**P2**"
            else:
                (x,y) = self.pol2cart(rho,phi)

            
        self.crane_pub = rospy.Publisher('hydrophones/crane_pos', Crane_pos, queue_size = 1)
        self.crane_pub.publish(Crane_pos(
            header=Header(stamp=rospy.Time.now(),
                          frame_id='Crane_pos_calc'),
            x_pos=x,
            y_pos=y,
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
        self.cardinal_pub = rospy.Publisher('hydrophones/cardinal', Float32, queue_size = 1)

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
