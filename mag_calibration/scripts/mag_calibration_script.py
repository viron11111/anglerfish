#!/usr/bin/python

import time
import sys
import math
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import MagneticField

class calibrate_mag():

	def min_max(self, data):
		self.x_out = data.magnetic_field.x
		self.y_out = data.magnetic_field.y
		self.z_out = data.magnetic_field.z

		if self.x_out > self.max_x:
			self.max_x = self.x_out
		elif self.x_out < self.min_x:
			self.min_x = self.x_out

		if self.y_out > self.max_y:
			self.max_y = self.y_out
		elif self.y_out < self.min_y:
			self.min_y = self.y_out

		if self.z_out > self.max_z:
			self.max_z = self.z_out
		elif self.z_out < self.min_z:
			self.min_z = self.z_out

		self.x_out_hard = self.x_out - ((self.max_x + self.min_x)/2.0)
		self.y_out_hard = self.y_out - ((self.max_y + self.min_y)/2.0)
		self.z_out_hard = self.z_out - ((self.max_z + self.min_z)/2.0)

	def __init__(self):
		self.dynamic_pub = rospy.Publisher("/imu/mag", MagneticField, queue_size=1)
		rospy.Subscriber("/imu/mag_raw", MagneticField, self.min_max)

		self.max_x = 0.0
		self.min_x = 0.0
		self.max_y = 0.0
		self.min_y = 0.0
		self.max_z = 0.0
		self.min_z = 0.0

		self.x_out = 0.0
		self.y_out = 0.0
		self.z_out = 0.0

		self.x_out_hard = 0.0
		self.y_out_hard = 0.0
		self.z_out_hard = 0.0

		rate = rospy.Rate(75)

		while not rospy.is_shutdown():

			mag = MagneticField(header = Header(
	                stamp = rospy.get_rostime(),
	                frame_id = 'magnetometer_corrected'
	            ),
	            magnetic_field = Vector3(self.x_out_hard, self.y_out_hard, self.z_out_hard)
	        )
	        self.dynamic_pub.publish(mag)
	        rate.sleep()

def main(args):
	rospy.init_node('mag_calibrator', anonymous=False)

	calibrate_mag()

	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		print "Shutting down"
		pass


if __name__ == '__main__':
	main(sys.argv)