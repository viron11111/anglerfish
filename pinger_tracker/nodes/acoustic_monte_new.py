#!/usr/bin/env python
import rospy
import rosparam
import random
import numpy as np
from matplotlib.mlab import griddata
import matplotlib.pyplot as plt
import math
import os
import cmath

from std_msgs.msg import Header, Bool
from pinger_tracker.msg import *
from pinger_tracker.srv import *

import dynamic_reconfigure.client

import csv
from time import localtime,strftime
import datetime
import time

#def callback(config):
    #rospy.loginfo("Config changed")

class monte(object):

    def sampling_rate(self,data):
        return Sample_rateResponse(self.sample_rate)

    def position_service(self, data):
        return Actual_positionResponse(self.position)

    def location_service(self, data):

        #can change x1, x2, x3, y2, y3
 
        #equilateral triangle layout
        '''hyp = 50.8
        var_a = hyp/2
        hydro0_xyz = [0,      0,     0]
        hydro1_xyz = [2*var_a,   0,     0]
        hydro2_xyz = [-var_a,  -var_a*np.sqrt(3),     0]
        hydro3_xyz = [-var_a,  var_a*np.sqrt(3), 0]'''

        #MIL T-shape layout
        #hydro0_xyz = [0,      0,     0]
        #hydro1_xyz = [100,   0,     0]
        #hydro2_xyz = [-100,  0,     0]
        #hydro3_xyz = [0,  -100, 0]      

        # Equilateral layout (actual)
        '''hydro0_xyz = [0,      0,     0]
        hydro1_xyz = [101.6,   0,     0]
        hydro2_xyz = [-50.8,  88.0,     0]
        hydro3_xyz = [-50.8,  -88.0, 0]  '''

        #experimental layout
        hydro0_xyz = [0,      0,     0]
        hydro1_xyz = [173.2,   0,     0]
        hydro2_xyz = [86.6,  150,     0]
        hydro3_xyz = [86.6,  50, -100] 

        return Hydrophone_locations_serviceResponse(hydro0_xyz, hydro1_xyz, hydro2_xyz ,hydro3_xyz)
    
    def calculate_error(self, x, y, z):

        self.position = [x,y,z]
        
    def calculate_error1(self, x, y, z):

        self.position = [x,y,z]

        ref = rospy.ServiceProxy('/hydrophones/crane_srv', Crane_pos_service)
        ref = ref()
        #print ref

        self.crane_x = ref.x
        self.crane_y = ref.y
        self.crane_z = ref.z

        ref2 = rospy.ServiceProxy('/hydrophones/actual_position', Actual_position)
        ref2 = ref2()

        self.actual_x = ref2.actual_position[0]
        self.actual_y = ref2.actual_position[1]
        self.actual_z = ref2.actual_position[2]  

        #os.system('clear')
        crane_heading = np.arctan2(self.crane_y,self.crane_x) + np.pi

        '''if self.crane_x != 0:
            crane_heading = np.arctan2(self.crane_y,self.crane_x) + np.pi
        else:
            crane_heading = 0.0'''
        #print "calculated_heading: %f radians" % crane_heading

        actual_heading = np.arctan2(self.actual_y,self.actual_x)+ np.pi
        '''print actual_heading

        if self.actual_x != 0:
            actual_heading = np.arctan2(self.actual_y,self.actual_x)+ np.pi
        else:
            actual_heading = 0.0'''
        print "crane: %f actual: %f" % (crane_heading, actual_heading)


        if self.crane_x == 0 and self.crane_y == 0:
            heading_error_radian = np.pi
            heading_error_percent = 50.0
        elif abs(actual_heading-(crane_heading + 2*np.pi)) < abs(actual_heading - crane_heading):
            heading_error_radian = abs(actual_heading-(crane_heading + 2*np.pi))
            heading_error_percent = abs((actual_heading-(crane_heading + 2*np.pi))/(2*np.pi)*100)
        else:
            heading_error_radian = abs(actual_heading - crane_heading)
            heading_error_percent = abs((actual_heading-crane_heading)/(2*np.pi)*100)

        self.head_error = heading_error_radian
        if self.head_error >= 6.0:
            print "{}\theading_error: %0.4f radians {}%f%%{}".format(self.W,self.O,self.W) % (heading_error_radian, heading_error_percent)
        #print "{}\theading_error: %0.4f radians {}%f%%{}".format(self.W,self.O,self.W) % (heading_error_radian, heading_error_percent)

        crane_horizontal_distance = np.sqrt(self.crane_x**2+self.crane_y**2)
        actual_horizontal_distance = np.sqrt(self.actual_x**2+self.actual_y**2)

        if crane_horizontal_distance != 0:
            calculated_declination = np.arctan(self.crane_z/crane_horizontal_distance)
        else:
            calculated_declination = 0.0

        if actual_horizontal_distance != 0:
            actual_declination = np.arctan(self.actual_z/actual_horizontal_distance)
        else:
            actual_declination = 0.0

        #print "crane: %f actual: %f" % (calculated_declination, actual_declination)
        

        #print "calculated_declination: %f radians" % calculated_declination
        #print "actual_declination: %f radians" % actual_declination


        '''if abs(actual_declination-(calculated_declination + 2*np.pi)) < abs(actual_declination - calculated_declination):
            declination_error_radian = abs(actual_declination-(calculated_declination + 2*np.pi))
            declination_error_percent = abs((actual_declination-(calculated_declination + 2*np.pi))/(2*np.pi)*100)
        else:'''

        declination_error_radian = abs(actual_declination - calculated_declination)
        declination_error_percent = abs((actual_declination - calculated_declination)/(2*np.pi)*100)

        #print declination_error_radian
        self.declination_error = declination_error_radian
        #declination_error_percent = abs((actual_declination-calculated_declination)/(2*np.pi)*100)
        #declination_error_radian = abs(actual_declination - calculated_declination)
        #print "{}\tdeclination_error: %0.4f radians {}%f%%{}".format(self.W,self.O,self.W) % (declination_error_radian, declination_error_percent)

        crane_distance = np.sqrt(self.crane_x**2+self.crane_y**2+self.crane_z**2)
        actual_distance = np.sqrt(self.actual_x**2+self.actual_y**2+self.actual_z**2)
        #print "calculated_distance %f" % crane_distance
        #print "actual_distance %f" % actual_distance

        distance_difference = actual_distance-crane_distance
        if actual_distance != 0.0:
            if crane_distance >= 30000:
                crane_distance = 0
            distance_error = (1.0 - (crane_distance/actual_distance))*100
            if distance_error > 100 or distance_error < -100:
                print "diserr: %f, crane_distance: %f, actual_distance: %f" % (distance_error, crane_distance, actual_distance)
        else:
            distance_error = 0.0

        self.heading_error_sum = self.heading_error_sum + heading_error_radian
        self.declination_error_sum = self.declination_error_sum + declination_error_radian
        self.distance_error_sum = self.distance_error_sum + distance_error

        #print "{}\tdistance_error: %.4f meters {}%.4f%%{}".format(self.W,self.O,self.W) % (distance_difference, distance_error)

        #print "***********************************"

    def plot_grid_graph(self,x_list,y_list,z,z_list,typemeasure):
        # define grid.



        xi = np.linspace(-self.max_range/1000, self.max_range/1000, (self.max_range/1000)*2)
        yi = np.linspace(-self.max_range/1000, self.max_range/1000, (self.max_range/1000)*2)
        npts = len(x_list)

        x_list = [x / 1000 for x in x_list]
        y_list = [y / 1000 for y in y_list]

        # grid the data.
        #print "x_list: %i y_list: %i xi: %i yi: %i" %(len(x_list), len(y_list), len(xi), len(yi))
        zi = griddata(x_list, y_list, z_list, xi, yi, interp='nn')
        # contour the gridded data, plotting dots at the nonuniform data points.
        if typemeasure == 'Heading':
            levels = [0,0.05,0.1,0.15,0.2,0.25,0.3,0.35,0.4,0.45,0.5,0.55,0.6,0.65,0.7,0.75,0.8,0.85,0.9,0.95,1.0,2.0, 3.0, 4.0, 5.0, 6.0, 6.28]
        elif typemeasure == "Declination":
            levels = 15#[-1,0,0.05,0.1,0.15,0.2,0.25,0.3,0.35,0.4,0.45,0.5,0.75,1.0,6.30]
        CS = plt.contour(xi, yi, zi, 5, linewidths=0.5, colors='k')

        if typemeasure == 'Heading':
            #vmax = 1.04
            vmax = 6.28
            vmin = 0
        elif typemeasure == 'Declination':
            vmax=abs(zi).max()
            vmin=-abs(zi).max()

        CS = plt.contourf(xi, yi, zi, levels, #100,
                          vmax=vmax, vmin=vmin)
                          #vmax=abs(zi).max(), vmin=-abs(zi).max())

        cb = plt.colorbar()
        cb.set_label(label='%s Error (Radians)' % typemeasure)#,size=18)
        # plot data points.
        #plt.scatter(x, y, marker='o', s=5, zorder=10)
        ref = rospy.ServiceProxy('/hydrophones/hydrophone_position', Hydrophone_locations_service)
        ref = ref()

        plt.plot([ref.hydro0_xyz[0]/100, ref.hydro1_xyz[0]/100, ref.hydro2_xyz[0]/100, ref.hydro3_xyz[0]/100], 
            [ref.hydro0_xyz[1]/100, ref.hydro1_xyz[1]/100, ref.hydro2_xyz[1]/100, ref.hydro3_xyz[1]/100], 'wo')
        plt.plot([ref.hydro1_xyz[0]/100, ref.hydro2_xyz[0]/100, ref.hydro3_xyz[0]/100], 
            [ref.hydro1_xyz[1]/100, ref.hydro2_xyz[1]/100, ref.hydro3_xyz[1]/100], 'ko', markersize = 3)
        plt.plot([ref.hydro0_xyz[0]/100], [ref.hydro0_xyz[1]/100], 'yo', markersize = 3)

        plt.xlim(-self.max_range/1000, self.max_range/1000)
        plt.ylim(-self.max_range/1000, self.max_range/1000)
        z = abs(z/1000)
        plt.ylabel('Meters')#,size=18)
        plt.xlabel('Meters')#,size=18)
        figure_title = 'Pinger Location VS %s Accuracy' % typemeasure
        figure_sub_title = '%i k/S/s sample rate (%d points) at depth %i meter(s)' % (self.sample_rate,npts,z)
        

        plt.suptitle('%s\n%s' % (figure_title,figure_sub_title), weight = 'bold', size = 14, x = 0.46, y = 1.01, horizontalalignment='center')

        plt.savefig('Tshape_high_error_resolution_contours_%s_%i_d%i_s%i.png' % (typemeasure,self.sample_rate,z,npts), dpi=300,
                     orientation = 'landscape', bbox_inches='tight')
        plt.show()
        plt.clf()
        plt.close()

    def __init__(self):

        self.client = dynamic_reconfigure.client.Client("signal_simulator_trigger", timeout=10)
        
        
        #self.toggle = rospy.Publisher('hydrophones/signal_trigger', Bool, queue_size = 1)
        
        #rospy.Subscriber('hydrophones/ping', Ping, self.actual_position)
        #rospy.Subscriber('hydrophones/crane_pos', Crane_pos, self.crane)

        rospy.Service('hydrophones/hydrophone_position', Hydrophone_locations_service, self.location_service)
        rospy.Service('hydrophones/actual_position', Actual_position, self.position_service)
        rospy.Service('hydrophones/sample_rate', Sample_rate, self.sampling_rate)
        #rospy.Service('hydrophones/ping', Ping_service, self.sampling_rate)

        self.actual_x = 0
        self.actual_y = 0
        self.actual_z = 0
        self.crane_x = 0
        self.crane_y = 0
        self.crane_z = 0

        self.W  = '\033[0m'  # white (normal)
        self.R  = '\033[31m' # red
        self.G  = '\033[32m' # green
        self.O  = '\033[43m' # orange
        self.B  = '\033[34m' # blue
        self.P  = '\033[35m' # purple      

        self.position = [3000.0, 3000.0, -1000.0]
        self.sample_rate = 1000

        self.head_error = 0
        self.declination_error = 0
        x_list = []
        y_list = []
        z_list = []
        d_list = []

        rate = rospy.Rate(1)  #rate of signals, 5 Hz for Anglerfish

        trigger = 0

        #date_time = strftime("%y_%m_%d_%H_%M_%S", localtime())
        #file_name = "/home/andy/catkin_ws/src/anglerfish/pinger_tracker/data/Sample_rate_results_%s.csv" % date_time

        z = -2000

        #self.file = csv.writer(open(file_name,'w'))
        #self.file.writerow(["Sample Rate", "Heading Error (rad)", "Declination Error (rad)", "Distance Error percent", "Depth: %i mm" % z])

        samples = 0
        self.heading_error_sum = 0.0
        self.declination_error_sum = 0.0
        self.distance_error_sum = 0.0

        resolution = 10000  

        self.xdistancemin = -10000 #in mm
        self.xdistancemax = 10000 #in mm
        self.ydistancemin =  -10000 #in mm
        self.ydistancemax = 10000 #inmm    

        #**********for polar coors****************

        self.sample_rate = 2000
        z = -1000 #depth of pinger

        self.max_range = 10000
        distance_resolution = 5000
        degree_angle_resolution = 10
        rad_resolution = math.radians(degree_angle_resolution)

        number_of_steps_per_rev = 360.0/degree_angle_resolution
        number_of_rings = self.max_range/distance_resolution
        total_samples = number_of_steps_per_rev * number_of_rings

        print rad_resolution
        print number_of_rings
        print "total samples %i" % total_samples

        for dis in range(distance_resolution,self.max_range+distance_resolution, distance_resolution):
            for deg in range(0,int(number_of_steps_per_rev)):
                phi = deg*rad_resolution
                x = dis * np.cos(phi)
                y = dis * np.sin(phi) 
                print "x: %f y: %f" % (x,y)

                self.calculate_error(x,y,z)

                ping = rospy.ServiceProxy('/hydrophones/ping_sim', Ping_service)

                x_list = x_list + [x]
                y_list = y_list + [y]
                z_list = z_list + [self.head_error]
                d_list = d_list + [self.declination_error]
                #time.sleep(1.0)

        self.plot_grid_graph(x_list,y_list,z,z_list,'Heading')
        self.plot_grid_graph(x_list,y_list,z,d_list,'Declination')       

        #****************polar coors**********************         

                #print dis


        '''for x in range(self.xdistancemin,self.xdistancemax+1,resolution):
            for y in range(self.ydistancemin,self.ydistancemax+1,resolution):
                self.sample_rate = 1000               

                z = -1000 #depth of pinger
                self.calculate_error(x,y,z)

                print x

                x_list = x_list + [x]
                y_list = y_list + [y]
                z_list = z_list + [self.head_error]
                d_list = d_list + [self.declination_error]

        self.plot_grid_graph(x_list,y_list,z,z_list,'Heading')
        self.plot_grid_graph(x_list,y_list,z,d_list,'Declination')'''


        #while not rospy.is_shutdown():

            #rate.sleep()




        '''for j in range(100, 10000, 10):
            #print j
            for x in range(-20000,20001,resolution):
                #print x
                for y in range(-20000,20001,resolution):
                    #print y
                    self.sample_rate = j                
                    #x = x*1000
                    #y = y*1000
                    z = -2000

                    self.calculate_error(x,y,z)
                    samples += 1
            heading_error = self.heading_error_sum/samples
            declination_error = self.declination_error_sum/samples
            distance_error = self.distance_error_sum/samples
            print "%i000 sampling rate, heading_error: %.2f rad, declination_error: %.2f rad, distance_error: %0.2f%% with %i samples" %(j,heading_error, declination_error, distance_error, samples)
            self.file.writerow(["%i000" % j, "%.2f" % heading_error, "%.2f" % declination_error, "%.2f" % distance_error])            
            samples = 0
            self.heading_error_sum = 0.0
            self.declination_error_sum = 0.0
            self.distance_error_sum = 0.0 '''          

        os.system("rosnode kill acoustic_monte")            



def main():
    rospy.init_node('acoustic_monte', anonymous=False)

    #rospy.loginfo("Waiting for signal_simulator service...")

    #rospy.wait_for_service("/signal_simulator/set_parameters")

   #rospy.loginfo("Beginning Monte")



    monte()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print "Shutting down"
        pass
    


if __name__ == '__main__':
    main()