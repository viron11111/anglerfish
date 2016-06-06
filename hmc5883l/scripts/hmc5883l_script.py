#!/usr/bin/python

import sys
import smbus
import time
import math
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import MagneticField



class measure_headings():

	def __init__(self):
		self.bus = smbus.SMBus(1)

		self.HMC5883L_ADDR       = 0x1E  #HMC5883L address

		self.mag_pub = rospy.Publisher("magnetic_headings", MagneticField, queue_size=1)



def main(args):
	rospy.init_node('hmc5883l_magnetometer', anonymous=False)

	measure_headings()

        try:
		rospy.spin()
        except rospy.ROSInterruptException:
		print "Shutting down"
                pass
	


if __name__ == '__main__':
	main(sys.argv)