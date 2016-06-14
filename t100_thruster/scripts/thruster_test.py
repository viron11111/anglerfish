#!/usr/bin/python

import sys
import time
import math
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Header
from std_msgs.msg import String
from anglerfish.msg import t100_thruster_feedback

class start_test():

	def forward(self):
		for i in range (0, 101, 1):
			print (float(i)/100.0)*2.36
			time.sleep(.01)

	def __init__(self):
		self.thrust1_pub = rospy.Publisher('/thruster1_force', Float32, queue_size=1)

		self.forward()

def main(args):
	rospy.init_node('thruster_test', anonymous=False)

	start_test()

        try:
		rospy.spin()
        except rospy.ROSInterruptException:
		print "Shutting down"
                pass
	


if __name__ == '__main__':
	main(sys.argv)
