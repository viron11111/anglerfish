#!/usr/bin/env python

import numpy as np
import rospy
import geometry_msgs.msg 
from std_msgs.msg import Header
from std_msgs.msg import Float32

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, Pose, Vector3, Point

import visualization_msgs.msg as visualization_msgs
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

import binascii
import struct
import encodings
import math
import tf, tf2_ros

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

        self.draw_sphere()

    def draw_sphere(self, scaling=(0.11, 0.11, 0.11), _id=4, frame='/front_stereo'): 

        markerArray = MarkerArray()

        marker = Marker()
        marker.header.frame_id = "/neck"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0

        self.rviz_pub.publish(marker)        

    def __init__(self):      
        
        rospy.init_node('pinger_visualizer')
        topic="visualization/markers"
        
        self.rviz_pub = rospy.Publisher(topic, visualization_msgs.Marker, queue_size=3)

        #rospy.Subscriber('/hydrophones/ping', Ping, self.parse_ping)

        rate = rospy.Rate(2)  #rate of signals, 5 Hz for Anglerfish

        while not rospy.is_shutdown():

            self.base_link()

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