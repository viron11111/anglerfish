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
		self.thruster1_x_offset = 0.00302616*math.pow(x,3) + 0.03098*math.pow(x,2) - 0.0013980*x + 0.001398
		self.thruster1_y_offset = -0.0002168*math.pow(x,3) + 0.005372*math.pow(x,2) + 0.00022265*x + 0.0002505573
		self.thruster1_z_offset = -0.000693*math.pow(x,2) + 0.0003195*x - 0.0003237

	def thruster_2_cal(self, data):
		x = data.data
		self.thruster2_x_offset = - 0.000685*math.pow(x,3) - 0.023506*math.pow(x,2) + 0.0008196*x - 0.0006908
		self.thruster2_y_offset = - 0.0002858*math.pow(x,2) + 0.00002892*x - 0.0001789
		self.thruster2_z_offset = 0.0002015*math.pow(x,2) + 0.000357*x - 0.00000623

	def thruster_3_cal(self, data):
		x = data.data
		self.thruster3_x_offset = - 0.00040896*math.pow(x,3) - 0.0145272*math.pow(x,2) + 0.00044587*x - 0.0003694
		self.thruster3_y_offset = -0.003212*math.pow(x,2) - 0.000323*x - 0.000214
		self.thruster3_z_offset = 0.00570201*math.pow(x,2) + 0.0002538*x - 0.0000405

	def thruster_4_cal(self, data):
		x = data.data
		self.thruster4_x_offset = 0.025143*math.pow(x,2) - 0.0006295*x + 0.000418
		self.thruster4_y_offset = 0.0007326*math.pow(x,2) + 0.0003393*x + 0.00006071
		self.thruster4_z_offset = -0.008161*math.pow(x,2) - 0.00006552*x - 0.00001022

	def thruster_5_cal(self, data):
		x = data.data
		self.thruster5_x_offset = 0.019102*math.pow(x,2) - 0.0005832*x + 0.0004361
		self.thruster5_y_offset = -0.0052331*math.pow(x,2) - 0.0001203*x - 0.0001762
		self.thruster5_z_offset = -0.01049994*math.pow(x,2) + 0.000020388*x - 0.0001353

	def thruster_6_cal(self, data):
		x = data.data
		self.thruster6_x_offset = -0.021234*math.pow(x,2) - 0.0000272*x - 0.00101
		self.thruster6_y_offset = 0.0006144*math.pow(x,2) - 0.0006081*x + 0.000146
		self.thruster6_z_offset = -0.006943*math.pow(x,2) + 0.0002278*x + 0.000185

	def min_max(self, data):
		self.x_out = data.magnetic_field.x
		self.y_out = data.magnetic_field.y
		self.z_out = data.magnetic_field.z

		scale = np.array([[1.051523805884331, -0.020851706173945314, 0.01997186678167343],
			              [-0.020851706173945314, 1.0577586404365373, 0.018433880871105954], 
			              [0.019971866781673394, 0.018433880871105965, 0.9001379227195077]])

		self.corrected = np.dot([self.x_out, self.y_out, self.z_out],scale) + np.array([0.0057537116805245515, 
			                                                                           -0.004091635490622386,
			                                                                            0.01347351737044513])

		


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

		self.corrected = np.array([0,0,0])

		self.x_out_hard = 0.0
		self.y_out_hard = 0.0
		self.z_out_hard = 0.0

		self.x_simple_cal = .04
		self.y_simple_cal = .04
		self.z_simple_cal = .04

		self.calibrated_values = (0.0, 0.0, 0.0)

		self.roll = 0.0
		self.pitch = 0.0
		self.yaw = 0.0

		rate = rospy.Rate(75)

		while not rospy.is_shutdown():
			'''magval.header = Header(
			    stamp = rospy.get_rostime(),
			    frame_id = 'magnetometer_comp_values')
	    	        
			magval.mag_pre_comp_x = self.x_out_hard
			magval.mag_pre_comp_y = self.y_out_hard
			magval.mag_pre_comp_z = self.z_out_hard
			magval.comp_roll = self.roll
			magval.comp_pitch = self.pitch
			magval.comp_yaw = self.yaw'''


			mag = MagneticField(header = 
          	        Header(stamp = rospy.get_rostime(),
                	frame_id = 'base_link'),
	                magnetic_field = Vector3(self.corrected[0], self.corrected[1], self.corrected[2]),
	                magnetic_field_covariance = [ 0.01, 0.0, 0.0,
                                                  0.0, 0.01, 0.0,
                                                  0.0, 0.0, 0.01])


			#self.mag_vals_pub.publish(magval)
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
