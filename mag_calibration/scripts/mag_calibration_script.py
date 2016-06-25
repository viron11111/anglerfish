#!/usr/bin/python

import time
import sys
import math
import rospy
import numpy as np
from std_msgs.msg import Float64, Float32
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import MagneticField
from t100_thruster.msg import t100_thruster_feedback
from mag_calibration.msg import mag_values
#from mag_calibration.msg import mag_values
#from anglerfish.msg import t100_thruster_feedback

class calibrate_mag():

	def thruster_1_cal(self, data):
		x = data.data
		self.thruster1_x_offset = 0.0073*math.pow(x,4) - 0.0034*math.pow(x,3) - 0.0275*math.pow(x,2) + 0.0027*x + 0.00005
		self.thruster1_y_offset = 0.0356*math.pow(x,6) - 0.0015*math.pow(x,5) - 0.0572*math.pow(x,4) + 0.0033*math.pow(x,3) + 0.0386*math.pow(x,2) - 0.0016*x - 0.0005
		self.thruster1_z_offset = -0.0015*math.pow(x,2) - 0.0005*x - 0.0002

	def thruster_2_cal(self, data):
		x = data.data
		self.thruster2_x_offset = 0.0073*math.pow(x,4) - 0.0034*math.pow(x,3) - 0.0275*math.pow(x,2) + 0.0027*x + 0.00005
		self.thruster2_y_offset = 0.0083*math.pow(x,4) - 0.004*math.pow(x,3) - 0.0315*math.pow(x,2) + 0.0032*x - 0.0001
		self.thruster2_z_offset = -0.0049*math.pow(x,2) - 0.0004*x - 0.0008

	def thruster_3_cal(self, data):
		x = data.data
		self.thruster3_x_offset = 0.005*math.pow(x,4) - 0.0035*math.pow(x,3) - 0.0169*math.pow(x,2) + 0.0032*x - 0.0009
		self.thruster3_y_offset = 0.0019*math.pow(x,2) + 0.0025*x + 0.0013
		self.thruster3_z_offset = -0.0086*math.pow(x,4) + 0.0011*math.pow(x,3) + 0.0182*math.pow(x,2) - 0.0009*x + 0.0006

	def thruster_4_cal(self, data):
		x = data.data
		self.thruster4_x_offset = -0.0073*math.pow(x,4) + 0.0036*math.pow(x,3) + 0.0269*math.pow(x,2) - 0.0036*x + 0.0004
		self.thruster4_y_offset = 0.0019*math.pow(x,2) + 0.0007*x - 0.0004
		self.thruster4_z_offset = -0.0053*math.pow(x,2) + 0.0003*x + 0.000008

	def thruster_5_cal(self, data):
		x = data.data
		self.thruster5_x_offset = -0.0056*math.pow(x,4) + 0.0022*math.pow(x,3) + 0.0177*math.pow(x,2) - 0.0022*x + 0.0002
		self.thruster5_y_offset = -0.0005*math.pow(x,2) + 0.003*x + 0.0009
		self.thruster5_z_offset = 0.0056*math.pow(x,4) - 0.002*math.pow(x,3) - 0.0176*math.pow(x,2) + 0.002*x - 0.0004

	def thruster_6_cal(self, data):
		x = data.data
		self.thruster6_x_offset = -0.0146*math.pow(x,2) - 0.0003*x - 0.0004
		self.thruster6_y_offset = -0.0053*math.pow(x,4) + 0.0023*math.pow(x,3) + 0.0186*math.pow(x,2) - 0.0023*x + 0.00009
		self.thruster6_z_offset = -0.006*math.pow(x,2) + 0.0005*x - 0.0009

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

		self.x_bias = ((self.max_x + self.min_x)/2.0) #for calculating bias
		self.y_bias = ((self.max_y + self.min_y)/2.0) #not to be used again until new location
		self.z_bias = ((self.max_z + self.min_z)/2.0)

		#f = (self.max_x + self.max_y + self.max_z)/3.0
		#print f
		f = .050232

		#print '************'
		#print self.max_x
		#print self.max_y
		#print self.max_z

		x_thruster_offset = self.thruster1_x_offset + self.thruster2_x_offset + self.thruster3_x_offset + self.thruster4_x_offset + self.thruster5_x_offset + self.thruster6_x_offset
		y_thruster_offset = self.thruster1_y_offset + self.thruster2_y_offset + self.thruster3_y_offset + self.thruster4_y_offset + self.thruster5_y_offset + self.thruster6_y_offset
		z_thruster_offset = self.thruster1_z_offset + self.thruster2_z_offset + self.thruster3_z_offset + self.thruster4_z_offset + self.thruster5_z_offset + self.thruster6_z_offset
		
		self.x_out_hard = self.x_out - 0.011868
		self.y_out_hard = self.y_out - (-0.017066)
		self.z_out_hard = self.z_out - 0.002116
		
		self.x_simple_cal = self.x_out_hard * (f/0.070288) - x_thruster_offset
		self.y_simple_cal = self.y_out_hard * (f/0.034224) - y_thruster_offset
		self.z_simple_cal = self.z_out_hard * (f/0.046092) - z_thruster_offset

		self.roll  = math.atan2(self.y_out, self.z_out) 
	    if (self.roll < 0):
	        self.roll += 2 * math.pi

            self.pitch  = math.atan2(self.x_out, self.z_out) 
	    if (self.pitch < 0):
		self.pitch += 2 * math.pi

            self.yaw  = math.atan2(self.y_out, self.x_out) 
	    if (self.yaw < 0):
		self.yaw += 2 * math.pi


		#rospy.logwarn("comp: %f, orig: %f, diff: %f" % (self.x_simple_cal, self.x_out_hard, self.x_simple_cal - 0.009331))

		self.hard_vals = np.matrix('%f;%f;%f' % (self.x_out_hard,self.y_out_hard, self.z_out_hard))

		soft_matrix = np.matrix('22.18458769, 0.68136508, 0.39133311; 0.0, 22.4054685, -0.2618374; 0.0, 0.0,  24.14833212')

		self.calibrated_values = soft_matrix * self.hard_vals

		#print self.calibrated_values[0]
		


	def __init__(self):
		self.dynamic_pub = rospy.Publisher("/imu/mag", MagneticField, queue_size=1)
		self.mag_vals_pub = rospy.Publisher("/comp_mag_values", mag_values, queue_size=1)

		magval = mag_values()

		self.thruster1_x_offset = 0.0
		self.thruster1_y_offset = 0.0
		self.thruster1_z_offset = 0.0
		self.thruster2_x_offset = 0.0
		self.thruster2_y_offset = 0.0
		self.thruster2_z_offset = 0.0
		self.thruster3_x_offset = 0.0
		self.thruster3_y_offset = 0.0
		self.thruster3_z_offset = 0.0
		self.thruster4_x_offset = 0.0
		self.thruster4_y_offset = 0.0
		self.thruster4_z_offset = 0.0
		self.thruster5_x_offset = 0.0
		self.thruster5_y_offset = 0.0
		self.thruster5_z_offset = 0.0
		self.thruster6_x_offset = 0.0
		self.thruster6_y_offset = 0.0
		self.thruster6_z_offset = 0.0

        self.max_x = 0.0001
        self.min_x = 0.0
        self.max_y = 0.0001
        self.min_y = 0.0
        self.max_z = 0.0001
        self.min_z = 0.0

		rospy.Subscriber("/imu/mag_raw", MagneticField, self.min_max)
		rospy.Subscriber('/thruster1_force', Float32, self.thruster_1_cal)
		rospy.Subscriber('/thruster2_force', Float32, self.thruster_2_cal)
		rospy.Subscriber('/thruster3_force', Float32, self.thruster_3_cal)
		rospy.Subscriber('/thruster4_force', Float32, self.thruster_4_cal)
		rospy.Subscriber('/thruster5_force', Float32, self.thruster_5_cal)
		rospy.Subscriber('/thruster6_force', Float32, self.thruster_6_cal)
		
		self.x_out = 0.0
		self.y_out = 0.0
		self.z_out = 0.0

		self.x_bias = 0.0
		self.y_bias = 0.0
		self.z_bias = 0.0

		self.x_out_hard = 0.0
		self.y_out_hard = 0.0
		self.z_out_hard = 0.0

		self.x_simple_cal = .04
		self.y_simple_cal = .04
		self.z_simple_cal = .04

		self.calibrated_values = (0.0, 0.0, 0.0)

		rate = rospy.Rate(75)

		while not rospy.is_shutdown():
			magval.header = Header(
				stamp = rospy.get_rostime(),
			    frame_id = 'magnetometer_comp_values')
	    	        
			magval.mag_pre_comp_x = self.x_out_hard
			magval.mag_pre_comp_y = self.y_out_hard
			magval.mag_pre_comp_z = self.z_out_hard
			magval.comp_roll = self.roll
			magval.comp_pitch = self.pitch
			magval.comp_yaw = self.yaw


			mag = MagneticField(header = 
                Header(stamp = rospy.get_rostime(),
                frame_id = 'magnetometer_corrected'),
                magnetic_field = Vector3(self.x_simple_cal, self.y_simple_cal, self.z_simple_cal))

			self.mag_vals_pub.publish(magval)
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
