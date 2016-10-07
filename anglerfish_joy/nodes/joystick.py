#!/usr/bin/python

import rospy
import geometry_msgs.msg 
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from sensor_msgs.msg import Joy
import tf, tf2_ros
from nav_msgs.msg import Odometry

'''
axes:
[0] : Left thumb X-axis
[1] : Left thumb Y-axis
[2] : 
[3] : Right thumb X-axis
[4] : Right thumb Y-axis
[5] : 
[6] : Dpad x-axis
[7] : Dpad y-axis

buttons:
[0] :  A (green)
[1] :  B (red)
[2] :  X (blue)
[3] :  Y (yellow)
[4] :  LB 
[5] :  RB
[6] :  BACK
[7] :  START
[8] : 
[9] :  LEFT THUMB
[10] : RIGHT THUMB
'''

class joystick(object):

	def base_link(self, data):
		self.subposx = data.pose.pose.position.x
		self.subposy = data.pose.pose.position.y
		self.subposz = data.pose.pose.position.z
		self.subrotx = data.pose.pose.orientation.x
		self.subroty = data.pose.pose.orientation.y
		self.subrotz = data.pose.pose.orientation.z
		self.subrotw = data.pose.pose.orientation.w


	def action(self, data):
		br = tf2_ros.TransformBroadcaster()
		t = geometry_msgs.msg.TransformStamped()

		if data.buttons[0] == 1:
			t.transform.translation.x = self.subposx
			t.transform.translation.y = self.subposy
			t.transform.translation.z = self.subposz
			t.transform.rotation.x = self.subrotx
			t.transform.rotation.y = self.subroty
			t.transform.rotation.z = self.subrotz
			t.transform.rotation.w = self.subrotw

		#odom = Odometry()

		t.header.stamp = rospy.Time.now()
		t.header.frame_id = "odom"
		t.child_frame_id = "desired_position"
		br.sendTransform(t)	



	def __init__(self):

		rospy.Subscriber("joy", Joy, self.action)
		rospy.Subscriber("odometry/filtered", Odometry, self.base_link)


def main():
	rospy.init_node('joystick', anonymous=False)

	joystick()

	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		print "Shutting down"
		pass


if __name__ == '__main__':
	main() #sys.argv