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
		rospy.Subscriber("ms5837_pressure_sensor", ms5837, self.ms5837)

		self.rpy_pub = rospy.Publisher('rpy_msg', rpy_msg, queue_size=1)

		self.depth = 0.0        
		self.window_size = 5
		self.slider = [0] * self.window_size

	def stim300(self, data):
		

		br = tf2_ros.TransformBroadcaster()
		t = geometry_msgs.msg.TransformStamped()

		t.header.stamp = rospy.Time.now()
		t.header.frame_id = "world"
		t.child_frame_id = "stim300"
		t.transform.translation.x = 1.0
		t.transform.translation.y = 1.0
		t.transform.translation.z = self.depth
		t.transform.rotation.x = data.orientation.x
		t.transform.rotation.y = data.orientation.y
		t.transform.rotation.z = data.orientation.z
		t.transform.rotation.w = data.orientation.w

		br.sendTransform(t)

		quaternion = (
			data.orientation.x,
			data.orientation.y,
			data.orientation.z,
			data.orientation.w)

		roll_pitch_yaw = tf.transformations.euler_from_quaternion(quaternion)

		rpy = rpy_msg()

		rpy.header = Header(
				stamp = rospy.get_rostime(),
				frame_id = str("roll_pitch_yaw")
			)

		rpy.roll = roll_pitch_yaw[0]
		rpy.pitch = roll_pitch_yaw[1]
		rpy.yaw = roll_pitch_yaw[2]

		self.rpy_pub.publish(rpy)


	def razor(self, data):

		br = tf2_ros.TransformBroadcaster()
		t = geometry_msgs.msg.TransformStamped()

		t.header.stamp = rospy.Time.now()
		t.header.frame_id = "world"
		t.child_frame_id = "razor"
		t.transform.translation.x = 1.0
		t.transform.translation.y = 0.5
		t.transform.translation.z = self.depth
		
		#q = tf.transformations.euler_from_quaternion(data.orientation)
		#print data.orientation
		#tf.transform(data.orientation, )
		t.transform.rotation.x = data.orientation.x
		t.transform.rotation.y = data.orientation.y
		t.transform.rotation.z = (data.orientation.z)#math.sqrt(0.5)
		t.transform.rotation.w = (data.orientation.w)#math.sqrt(0.5)

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
