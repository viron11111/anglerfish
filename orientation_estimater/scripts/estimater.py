#!/usr/bin/python

import serial
import numpy as np
import rospy
import geometry_msgs.msg 
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from std_msgs.msg import Float32
from ms5837.msg import ms5837 
from orientation_estimater.msg import rpy_msg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, PoseWithCovarianceStamped

import binascii
import struct
import encodings
import math
import tf, tf2_ros



class Interface(object):

	def ms5837(self, data):
		for i in range(self.window_size-1, -1, -1):
			self.slider[i] = self.slider[i-1]
		self.slider[0] = data.depth
		self.depth = sum(self.slider)/self.window_size
		#print self.depth

	def __init__(self):

		#self.pose_pub = rospy.Publisher("/static_point", PoseStamped, queue_size = 0)
		rospy.Subscriber("imu/data", Imu, self.stim300)
		rospy.Subscriber("imu/razor", Imu, self.razor)
		rospy.Subscriber("depth", PoseWithCovarianceStamped, self.pressure)
		rospy.Subscriber("odometry/filtered", Odometry, self.base_link)

		self.depth = 0.0        
		self.window_size = 5
		self.slider = [0] * self.window_size

	def pressure(self, data):
                br = tf2_ros.TransformBroadcaster()
                t = geometry_msgs.msg.TransformStamped()

                t.header.stamp = rospy.Time.now()
                t.header.frame_id = "base_link"
                t.child_frame_id = "pressure"
                t.transform.translation.x = -0.167 
                t.transform.translation.y = 0.015 
                t.transform.translation.z = -0.015 
                t.transform.rotation.x = 0
                t.transform.rotation.y = 0
                t.transform.rotation.z = 0
                t.transform.rotation.w = 1.0

                br.sendTransform(t)


	def base_link(self, data):
                br = tf2_ros.TransformBroadcaster()
                t = geometry_msgs.msg.TransformStamped()
                
		#odom = Odometry()

		t.header.stamp = rospy.Time.now()
                t.header.frame_id = "odom"
                t.child_frame_id = "base_link"
                t.transform.translation.x = data.pose.pose.position.x 
                t.transform.translation.y = data.pose.pose.position.y 
                t.transform.translation.z = data.pose.pose.position.z 
                t.transform.rotation.x = data.pose.pose.orientation.x
                t.transform.rotation.y = data.pose.pose.orientation.y
                t.transform.rotation.z = data.pose.pose.orientation.z
                t.transform.rotation.w = data.pose.pose.orientation.w

                br.sendTransform(t)

	def stim300(self, data):
		

		br = tf2_ros.TransformBroadcaster()
		t = geometry_msgs.msg.TransformStamped()

		t.header.stamp = rospy.Time.now()
		t.header.frame_id = "base_link"
		t.child_frame_id = "stim300"
		t.transform.translation.x = -0.085
		t.transform.translation.y = 0
		t.transform.translation.z = -0.031
		t.transform.rotation.x = 0
		t.transform.rotation.y = 0
		t.transform.rotation.z = 0
		t.transform.rotation.w = 1.0

		br.sendTransform(t)

		quaternion = (
			data.orientation.x,
			data.orientation.y,
			data.orientation.z,
			data.orientation.w)



	def razor(self, data):

		br = tf2_ros.TransformBroadcaster()
		t = geometry_msgs.msg.TransformStamped()

		t.header.stamp = rospy.Time.now()
		t.header.frame_id = "base_link"
		t.child_frame_id = "razor"
		t.transform.translation.x = 0.061
		t.transform.translation.y = -0.004
		t.transform.translation.z = 0.022
		
		t.transform.rotation.x = 0.0
		t.transform.rotation.y = 0.0
		t.transform.rotation.z = 0.0
		t.transform.rotation.w = 1.0

		br.sendTransform(t)

def main():
	rospy.init_node('orientation_node', anonymous=False)

	Interface()

	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		print "Shutting down"
		pass


if __name__ == '__main__':
	main() #sys.argv
