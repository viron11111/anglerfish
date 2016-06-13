#!/usr/bin/python

import rospy
from std_msgs.msg import Header
from std_msgs.msg import Float32, Int16
from orientation_estimater.msg import rpy_msg
import math

class stability(object):

	def __init__(self):

		rospy.Subscriber("rpy_msg", rpy_msg, self.stabilize)
		self.servo_pub = rospy.Publisher("servo_position", Int16, queue_size=1)

	def stabilize(self, data):

		output = int(-data.pitch*215 + 400)
		#rospy.logwarn(output)
		if output <= 140:
			output = 140
		elif output >= 710:
			ouput = 710
		#rospy.logwarn( output)
		self.servo_pub.publish(output)

def main():
	rospy.init_node('camera_stabilization_node', anonymous=False)

	stability()

	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		print "Shutting down"
		pass


if __name__ == '__main__':
	main() #sys.argv
