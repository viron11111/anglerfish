#!/usr/bin/env python
import rospy
import rosparam
import random
import numpy as np
import matplotlib.pyplot as plt
import math
import os

from std_msgs.msg import Header, Bool
from pinger_tracker.msg import *
from pinger_tracker.srv import *

import dynamic_reconfigure.client

#def callback(config):
    #rospy.loginfo("Config changed")

class monte(object):

    def position_service(self, data):
        position = [4000.0, 3000.0, -1000.0]
        return Actual_positionResponse(position)

    def crane(self, data):
        self.crane_x = data.x_pos/1000.0
        self.crane_y = data.y_pos/1000.0
        self.crane_z = data.z_pos/1000.0

    def change_position(self, position):
        client.update_configuration({"pinger_x_pos":rand_num})
        client.update_configuration({"pinger_y_pos":rand_num})
        client.update_configuration({"pinger_z_pos":rand_num})

    def location_service(self, data):

        hydro0_xyz = [0,     0,     0]
        hydro1_xyz = [-25.4, 0,     0]
        hydro2_xyz = [25.4,  0,     0]
        hydro3_xyz = [0,     -25.4, 0]

        return Hydrophone_locations_serviceResponse(hydro0_xyz, hydro1_xyz, hydro2_xyz ,hydro3_xyz)

    def calculate_error(self):

        ref = rospy.ServiceProxy('/hydrophones/crane_srv', Crane_solution)
        ref = ref()

        self.crane_x = ref.x
        self.crane_y = ref.y
        self.crane_z = ref.z

        ref2 = rospy.ServiceProxy('/hydrophones/actual_position', Actual_position)
        ref2 = ref2()

        self.actual_x = ref2.actual_position[0]
        self.actual_y = ref2.actual_position[1]
        self.actual_z = ref2.actual_position[2]        



        os.system('clear')
        if self.crane_x != 0:
            crane_heading = np.arctan2(self.crane_y,self.crane_x) + np.pi
        else:
            crane_heading = 0.0
        print "calculated_heading: %f radians" % crane_heading

        if self.actual_x != 0:
            actual_heading = np.arctan2(self.actual_y,self.actual_x)+ np.pi
        else:
            actual_heading = 0.0
        print "actual_heading: %f radians" % actual_heading

        heading_error_percent = abs((actual_heading-crane_heading)/(2*np.pi)*100)
        heading_error_radian = abs(actual_heading - crane_heading)
        print "{}\theading_error: %0.4f radians {}%f%%{}".format(self.W,self.O,self.W) % (heading_error_radian, heading_error_percent)

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

        print "calculated_declination: %f radians" % calculated_declination
        print "actual_declination: %f radians" % actual_declination

        declination_error_percent = abs((actual_declination-calculated_declination)/(2*np.pi)*100)
        declination_error_radian = abs(actual_declination - calculated_declination)
        print "{}\tdeclination_error: %0.4f radians {}%f%%{}".format(self.W,self.O,self.W) % (declination_error_radian, declination_error_percent)

        crane_distance = np.sqrt(self.crane_x**2+self.crane_y**2+self.crane_z**2)
        actual_distance = np.sqrt(self.actual_x**2+self.actual_y**2+self.actual_z**2)
        print "calculated_distance %f" % crane_distance
        print "actual_distance %f" % actual_distance

        distance_difference = actual_distance-crane_distance
        if actual_distance != 0.0:
            distance_error = (1.0 - (crane_distance/actual_distance))*100
        else:
            distance_error = 0.0

        print "{}\tdistance_error: %.4f meters {}%.4f%%{}".format(self.W,self.O,self.W) % (distance_difference, distance_error)

        print "***********************************"



    def __init__(self):

        #client = dynamic_reconfigure.client.Client("signal_simulator", timeout=30)
        #client.update_configuration({"signal_gen_trigger":'True'})
        
        #self.toggle = rospy.Publisher('hydrophones/signal_trigger', Bool, queue_size = 1)
        
        #rospy.Subscriber('hydrophones/ping', Ping, self.actual_position)
        #rospy.Subscriber('hydrophones/crane_pos', Crane_pos, self.crane)

        rospy.Service('hydrophones/hydrophone_position', Hydrophone_locations_service, self.location_service)
        rospy.Service('hydrophones/actual_position', Actual_position, self.position_service)

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

        rate = rospy.Rate(1)  #rate of signals, 5 Hz for Anglerfish

        trigger = 0

        while not rospy.is_shutdown():

            self.calculate_error()

            '''rand_num = random.uniform(25,50)

            self.toggle.publish(Bool(
                    data=trigger))

            trigger = not trigger'''

            rate.sleep()        



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