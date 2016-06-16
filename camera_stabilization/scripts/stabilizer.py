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

		self.angle = rospy.get_param('~angle', 345)

		self.output = 345

		self.rate = rospy.Rate(25)

		while not rospy.is_shutdown():
			self.servo_pub.publish(self.output)
			self.rate.sleep()

	def stabilize(self, data):

		self.output = int(-data.pitch*200 + self.angle)

		if self.output <= 140:
			self.output = 140
		elif self.output >= 710:
			self.ouput = 710

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
