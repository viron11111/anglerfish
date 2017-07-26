#!/usr/bin/env python

import numpy as np
import rospy
import geometry_msgs.msg 
from std_msgs.msg import Header
from std_msgs.msg import Float32

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, Pose, Vector3, Point

#import visualization_msgs.msg as visualization_msgs
from visualization_msgs.msg import Marker
#from visualization_msgs.msg import MarkerArray

import binascii
import struct
import encodings
import math
import tf, tf2_ros

from pinger_tracker.msg import *

class visualizer(object):

    def hydrophone_locations(self, data):
        self.hydro0 = [data.hydro0_xyz[0],data.hydro0_xyz[1],data.hydro0_xyz[2]]
        self.hydro1 = [data.hydro1_xyz[0],data.hydro1_xyz[1],data.hydro1_xyz[2]]
        self.hydro2 = [data.hydro2_xyz[0],data.hydro2_xyz[1],data.hydro2_xyz[2]]
        self.hydro3 = [data.hydro3_xyz[0],data.hydro3_xyz[1],data.hydro3_xyz[2]]    

    def base_link(self):
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "odom"
        t.child_frame_id = "SONAR"
        t.transform.translation.x = 0
        t.transform.translation.y = 0 
        t.transform.translation.z = 0
        t.transform.rotation.x = 0
        t.transform.rotation.y = 0
        t.transform.rotation.z = 0
        t.transform.rotation.w = 1.0

        br.sendTransform(t)

    def ls(self,data):
        marker1 = Marker()
        marker1.header.frame_id = "/map"
        marker1.header.stamp = rospy.Time.now()
        marker1.type = marker1.SPHERE
        marker1.action = marker1.ADD

        marker1.scale.x = 0.05
        marker1.scale.y = 0.05
        marker1.scale.z = 0.05

        if data.header.frame_id == "ls_pos_calc":
            marker1.ns = "LS_calc_position"
            marker1.id = 11

        elif data.header.frame_id == "ls_pos_actu":
            marker1.ns = "LS_actu_position"
            marker1.id = 12

        marker1.color.a = 1.0
        marker1.color.r = 0.0
        marker1.color.g = 0.0
        marker1.color.b = 1.0      

        marker1.lifetime = rospy.Duration()
        marker1.pose.orientation.x = 0.0
        marker1.pose.orientation.y = 0.0
        marker1.pose.orientation.z = 0.0
        marker1.pose.orientation.w = 1.0
        marker1.pose.position.x = (data.x_pos)/1000.0
        marker1.pose.position.y = (data.y_pos)/1000.0
        marker1.pose.position.z = (data.z_pos)/1000.0

        #print marker1

        self.publisher.publish(marker1)

        vector = Point(data.x_pos/1000.0, data.y_pos/1000.0, data.z_pos/1000.0)
        tail = Point(0,0,0)

        marker = Marker()
        marker.header.stamp = rospy.Time(0)
        marker.header.frame_id = '/map'

        if data.header.frame_id == "ls_pos_calc":
            marker.ns = "LS_calc_position"
            marker.id = 13

        if data.header.frame_id == "ls_pos_actu":
            marker.ns = "LS_actu_position"
            marker.id = 14

        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.r = 1.0
        marker.color.a = 0.5            

        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.lifetime = rospy.Duration()
        marker.points.append(tail)
        marker.points.append(vector)
        marker.scale.x = 0.05
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.5
        self.publisher.publish(marker)                

    def crane(self,data):
        marker1 = Marker()
        marker1.header.frame_id = "/map"
        marker1.header.stamp = rospy.Time.now()
        marker1.type = marker1.SPHERE
        marker1.action = marker1.ADD

        marker1.scale.x = 0.05
        marker1.scale.y = 0.05
        marker1.scale.z = 0.05

        if data.header.frame_id == "Crane_pos_calc":
            marker1.ns = "Crane_calc_position"
            marker1.id = 7

            marker1.color.a = 1.0
            marker1.color.r = 1.0
            marker1.color.g = 0.0
            marker1.color.b = 0.0
        elif data.header.frame_id == "Crane_pos_actu":
            marker1.ns = "Crane_actu_position"
            marker1.id = 8

            marker1.color.a = 1.0
            marker1.color.r = 1.0
            marker1.color.g = 0.0
            marker1.color.b = 0.0            


        marker1.lifetime = rospy.Duration()
        marker1.pose.orientation.x = 0.0
        marker1.pose.orientation.y = 0.0
        marker1.pose.orientation.z = 0.0
        marker1.pose.orientation.w = 1.0
        marker1.pose.position.x = (data.x_pos)/1000.0
        marker1.pose.position.y = (data.y_pos)/1000.0
        marker1.pose.position.z = (data.z_pos)/1000.0

        #print marker1

        self.publisher.publish(marker1)

        vector = Point(data.x_pos/1000.0, data.y_pos/1000.0, data.z_pos/1000.0)
        tail = Point(0,0,0)

        marker = Marker()
        marker.header.stamp = rospy.Time(0)
        marker.header.frame_id = '/map'

        if data.header.frame_id == "Crane_pos_calc":
            marker.ns = "Crane_calc_position"
            marker.id = 9
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.5
        if data.header.frame_id == "Crane_pos_actu":
            marker.ns = "Crane_actu_position"
            marker.id = 10
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.5            

        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.lifetime = rospy.Duration()
        marker.points.append(tail)
        marker.points.append(vector)
        marker.scale.x = 0.05
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.5
        self.publisher.publish(marker)        

    def bancroft(self, data):
        marker1 = Marker()
        marker1.header.frame_id = "/map"
        marker1.header.stamp = rospy.Time.now()
        marker1.type = marker1.SPHERE
        marker1.action = marker1.ADD

        marker1.scale.x = 0.05
        marker1.scale.y = 0.05
        marker1.scale.z = 0.05

        if data.header.frame_id == "bancroft_pos_calc":
            marker1.ns = "Bancroft_calc_position"
            marker1.id = 15


        elif data.header.frame_id == "bancroft_pos_actu":
            marker1.ns = "Bancroft_actu_position"
            marker1.id = 16
          
        marker1.color.a = 1.0
        marker1.color.r = 0.5
        marker1.color.g = 0.5
        marker1.color.b = 0.0 

        marker1.lifetime = rospy.Duration()
        marker1.pose.orientation.x = 0.0
        marker1.pose.orientation.y = 0.0
        marker1.pose.orientation.z = 0.0
        marker1.pose.orientation.w = 1.0
        marker1.pose.position.x = (data.x_pos)/1000.0
        marker1.pose.position.y = (data.y_pos)/1000.0
        marker1.pose.position.z = (data.z_pos)/1000.0

        #print marker1

        if marker1.id != 0:
            self.publisher.publish(marker1)

        vector = Point(data.x_pos/1000.0, data.y_pos/1000.0, data.z_pos/1000.0)
        tail = Point(0,0,0)

        marker = Marker()
        marker.header.stamp = rospy.Time(0)
        marker.header.frame_id = '/map'

        if data.header.frame_id == "bancroft_pos_calc":
            marker.ns = "Bancroft_calc_position"
            marker.id = 17

        if data.header.frame_id == "bancroft_pos_actu":
            marker.ns = "Bancroft_actu_position"
            marker.id = 18

        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.lifetime = rospy.Duration()
        marker.points.append(tail)
        marker.points.append(vector)
        marker.scale.x = 0.05
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.g = 0.0
        marker.color.b = 0.5
        marker.color.a = 0.5

        if marker.id != 0:
            self.publisher.publish(marker) 

    def actual_pos(self,data):

        marker1 = Marker()
        marker1.header.frame_id = "/map"
        marker1.header.stamp = rospy.Time.now()
        marker1.type = marker1.SPHERE
        marker1.action = marker1.ADD
        marker1.ns = "actual_position"
        marker1.id = 4

        marker1.scale.x = 0.1
        marker1.scale.y = 0.1
        marker1.scale.z = 0.1

        marker1.color.a = 1.0
        marker1.color.r = 0.0
        marker1.color.g = 1.0
        marker1.color.b = 0.0

        marker1.lifetime = rospy.Duration()
        marker1.pose.orientation.x = 0.0
        marker1.pose.orientation.y = 0.0
        marker1.pose.orientation.z = 0.0
        marker1.pose.orientation.w = 1.0
        marker1.pose.position.x = (data.actual_position[0])/1000.0
        marker1.pose.position.y = (data.actual_position[1])/1000.0
        marker1.pose.position.z = (data.actual_position[2])/1000.0

        #print marker1

        self.publisher.publish(marker1)

        

    def draw_hydrophones(self): 

        marker1 = Marker()
        marker1.header.frame_id = "/map"
        marker1.header.stamp = rospy.Time.now()
        marker1.type = marker1.CYLINDER
        marker1.action = marker1.ADD
        marker1.ns = "hydrophone_0"
        marker1.id = 0

        marker1.scale.x = 0.003
        marker1.scale.y = 0.003
        marker1.scale.z = 0.1

        marker1.color.a = 1.0
        marker1.color.r = 1.0
        marker1.color.g = 1.0
        marker1.color.b = 1.0

        marker1.lifetime = rospy.Duration()
        marker1.pose.orientation.x = 0.0
        marker1.pose.orientation.y = 0.0
        marker1.pose.orientation.z = 0.0
        marker1.pose.orientation.w = 1.0
        marker1.pose.position.x = self.hydro0[0]/1000.0
        marker1.pose.position.y = self.hydro0[1]/1000.0
        marker1.pose.position.z = self.hydro0[2]/1000.0

        self.publisher.publish(marker1)

        marker2 = marker1

        marker2.pose.position.x = self.hydro1[0]/1000.0
        marker2.pose.position.y = self.hydro1[1]/1000.0
        marker2.pose.position.z = self.hydro1[2]/1000.0
        marker2.ns = "hydrophone_1"
        marker2.id = 1
     
        self.publisher.publish(marker2)     

        marker3 = marker1

        marker3.pose.position.x = self.hydro2[0]/1000.0
        marker3.pose.position.y = self.hydro2[1]/1000.0
        marker3.pose.position.z = self.hydro2[2]/1000.0
        marker3.ns = "hydrophone_2"
        marker3.id = 2
     
        self.publisher.publish(marker3) 

        marker4 = marker1

        marker4.pose.position.x = self.hydro3[0]/1000.0
        marker4.pose.position.y = self.hydro3[1]/1000.0
        marker4.pose.position.z = self.hydro3[2]/1000.0
        marker4.ns = "hydrophone_3"
        marker4.id = 3
     
        self.publisher.publish(marker4)

    def __init__(self):      
        
        rospy.init_node('pinger_visualizer')

        topic = 'visualization_marker'
        self.publisher = rospy.Publisher(topic, Marker, queue_size=100)
        rospy.Subscriber('hydrophones/bancroft_pos', Bancroft_pos, self.bancroft)
        rospy.Subscriber('hydrophones/crane_pos', Crane_pos, self.crane)
        rospy.Subscriber('hydrophones/ls_pos', Ls_pos, self.ls)
        rospy.Subscriber('hydrophones/ping', Ping, self.actual_pos)
        rospy.Subscriber('hydrophones/hydrophone_locations', Hydrophone_locations, self.hydrophone_locations)
        #rospy.sleep(1)

        self.hydro0 = [0,     0,     0]
        self.hydro1 = [-25.4, 0,     0]
        self.hydro2 = [25.4,  0,     0]
        self.hydro3 = [0,     -25.4, 0]

        self.rates = 1

        rate = rospy.Rate(self.rates)  #rate of signals, 5 Hz for Anglerfish

        while not rospy.is_shutdown():

            #self.base_link()
            self.draw_hydrophones()

            rate.sleep()


def main():
    rospy.init_node('pinger_visualizer', anonymous=False)

    visualizer()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print "Shutting down"
        pass
    


if __name__ == '__main__':
    main()