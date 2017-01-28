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

	def min_max(self, data):
		self.x_out = data.magnetic_field.x
		self.y_out = data.magnetic_field.y
		self.z_out = data.magnetic_field.z

		scale = np.array([[1.0668297239276732, -0.013402138039921378, -0.014120127475188347],
			              [-0.013402138039921463, 1.0672934593219578, 0.0006424302836179077], 
			              [-0.014120127475188338, 0.000642430283618064, 0.8785814940921376]])

		self.corrected = np.dot([self.x_out, self.y_out, self.z_out],scale) + np.array([0.00610837387568588, 
			                                                                           0.0066839069662578705,
			                                                                            0.011252580642419059])

	def __init__(self):
		self.dynamic_pub = rospy.Publisher("/imu/mag", MagneticField, queue_size=1)
		self.mag_vals_pub = rospy.Publisher("/comp_mag_values", mag_values, queue_size=1)

		magval = mag_values()

		rospy.Subscriber("/imu/mag_raw", MagneticField, self.min_max)

		rate = rospy.Rate(75)

		while not rospy.is_shutdown():

			mag = MagneticField(header = 
          	        Header(stamp = rospy.get_rostime(),
                	frame_id = 'base_link'),
	                magnetic_field = Vector3(self.corrected[0], self.corrected[1], self.corrected[2]),
	                magnetic_field_covariance = [ 0.01, 0.0, 0.0,
                                                  0.0, 0.01, 0.0,
                                                  0.0, 0.0, 0.01])

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
