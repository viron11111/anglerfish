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

    def ls_pos(self,data):
        marker1 = Marker()
        marker1.header.frame_id = "/map"
        marker1.header.stamp = rospy.Time.now()
        marker1.type = marker1.SPHERE
        marker1.action = marker1.ADD
        marker1.ns = "ls_position"
        marker1.id = 5

        marker1.scale.x = 0.1
        marker1.scale.y = 0.1
        marker1.scale.z = 0.1

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
        marker.ns = "pinger_heading"
        marker.header.stamp = rospy.Time(0)
        marker.header.frame_id = '/map'
        marker.id = 6

        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.lifetime = rospy.Duration()
        marker.points.append(tail)
        marker.points.append(vector)
        marker.scale.x = 0.1
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.5
        self.publisher.publish(marker)

        
        #t = math.tan(vector[1]/vector[0])
        #p = 



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
        marker1.pose.position.x = 0
        marker1.pose.position.y = 0
        marker1.pose.position.z = 0

        self.publisher.publish(marker1)

        marker2 = marker1

        marker2.pose.position.x = -0.0254
        marker2.ns = "hydrophone_1"
        marker2.id = 1
     
        self.publisher.publish(marker2)     

        marker3 = marker1

        marker3.pose.position.x = 0.0254
        marker3.ns = "hydrophone_2"
        marker3.id = 2
     
        self.publisher.publish(marker3) 

        marker4 = marker1

        marker4.pose.position.y = -0.0254
        marker4.pose.position.x = 0
        marker4.ns = "hydrophone_3"
        marker4.id = 3
     
        self.publisher.publish(marker4)

    def __init__(self):      
        
        rospy.init_node('pinger_visualizer')

        topic = 'visualization_marker'
        self.publisher = rospy.Publisher(topic, Marker, queue_size=100)
        rospy.Subscriber('hydrophones/LS_pos', LS_pos, self.ls_pos)
        rospy.Subscriber('hydrophones/ping', Ping, self.actual_pos)
        #rospy.sleep(1)

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