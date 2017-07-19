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

#1600, -1600, -2000 (mm)
#0.0, 9.119770766119473, -9.016221156343818, -9.016221156343818

class lssolver():

    def crane(self,data):
        if data.header.frame_id == 'Crane_pos_calc':
            self.init_guess = [data.x_pos,data.y_pos,data.z_pos]    

    def hydrophone_locations(self, data):
        self.hydro0 = [data.hydro0_xyz[0],data.hydro0_xyz[1],data.hydro0_xyz[2]]
        self.hydro1 = [data.hydro1_xyz[0],data.hydro1_xyz[1],data.hydro1_xyz[2]]
        self.hydro2 = [data.hydro2_xyz[0],data.hydro2_xyz[1],data.hydro2_xyz[2]]
        self.hydro3 = [data.hydro3_xyz[0],data.hydro3_xyz[1],data.hydro3_xyz[2]]    

    def ls_calc_vals (self, data):
        x,y,z = self.ls_solver(data)

        self.ls_pub.publish(Ls_pos(
            header=Header(stamp=rospy.Time.now(),
                          frame_id='ls_pos_calc'),
            x_pos=x,
            y_pos=y,
            z_pos=z))

    def actu_vals (self, data):
        x,y,z = self.ls_solver(data)

        self.ls_pub.publish(Ls_pos(
            header=Header(stamp=rospy.Time.now(),
                          frame_id='ls_pos_actu'),
            x_pos=x,
            y_pos=y,
            z_pos=z))       

    def ls_solver(self, data):

        hydrophone_locations = np.array([self.hydro0, self.hydro1, self.hydro2, self.hydro3])

        c = 1.484  # millimeters/microsecond
        #hydrophone_array = ReceiverArraySim(hydrophone_locations, c)
        sonar = Multilaterator(hydrophone_locations, c, 'LS', self.init_guess)

        if data.header.frame_id == 'phase_shift':
            tstamps = data.calculated_time_stamps
        elif data.header.frame_id == 'signal_sim':
            tstamps = data.actual_time_stamps

        res_msg = sonar.get_pulse_location(np.array(tstamps))
        
        return res_msg

    def __init__(self):

        self.hydro0 = [0,     0,     0]
        self.hydro1 = [-25.4, 0,     0]
        self.hydro2 = [25.4,  0,     0]
        self.hydro3 = [0,     -25.4, 0]

        self.init_guess = np.random.normal(0,100,3)

        rospy.Subscriber('/hydrophones/actual_time_stamps', Actual_time_stamps, self.actu_vals)
        rospy.Subscriber('/hydrophones/calculated_time_stamps', Calculated_time_stamps, self.ls_calc_vals)
        rospy.Subscriber('hydrophones/hydrophone_locations', Hydrophone_locations, self.hydrophone_locations)
        rospy.Subscriber('hydrophones/crane_pos', Crane_pos, self.crane)

        self.ls_pub = rospy.Publisher('hydrophones/ls_pos', Ls_pos, queue_size = 1)

        rate = rospy.Rate(1)

        while not rospy.is_shutdown():

            rate.sleep()

def main():
    rospy.init_node('lsmethod', anonymous=False)

    lssolver()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print "Shutting down"
        pass
    


if __name__ == '__main__':
    main()              