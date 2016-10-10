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

	def base_link(self):#, data):
		self.base_link_posx = 0#data.pose.pose.position.x
		self.base_link_posy = 0#data.pose.pose.position.y
		self.base_link_posz = 0#data.pose.pose.position.z
		self.base_link_rotx = 0#data.pose.pose.orientation.x
		self.base_link_roty = 0#data.pose.pose.orientation.y
		self.base_link_rotz = 0#data.pose.pose.orientation.z
		self.base_link_rotw = 1.0#data.pose.pose.orientation.w

	def start(self):
		self.subposx = 0
		self.subposy = 0
		self.subposz = 0
		self.subrotx = 0
		self.subroty = 0
		self.subrotz = 0
		self.subrotw = 1.0


	def action(self, data):


		if data.buttons[1] == 1:
			self.subposx = self.base_link_posx
			self.subposy = self.base_link_posy
			self.subposz = self.base_link_posz
			self.subrotx = self.base_link_rotx
			self.subroty = self.base_link_roty
			self.subrotz = self.base_link_rotz
			self.subrotw = self.base_link_rotw
		elif data.buttons[2] == 1:
			self.subposz += .1
		elif data.buttons[0] == 1:
			self.subposz -= .1

		if data.axes[1] != 0.0:
			self.subposx += data.axes[1]*.01

		if data.axes[0] != 0.0:
			self.subposy += data.axes[0]*.01

		if data.axes[3] != 0.0:
			quat = (
				self.subrotx,
				self.subroty,
				self.subrotz,
				self.subrotw)

			euler = tf.transformations.euler_from_quaternion(quat)
			
			yaw = euler[2] + data.axes[3]*.1
			euler = (
				euler[0],
				euler[1],
				yaw)
			#rospy.loginfo(euler)

			quat = tf.transformations.quaternion_from_euler(euler[0],euler[1], euler[2])
			self.subrotx = quat[0]
			self.subroty = quat[1]
			self.subrotz = quat[2]
			self.subrotw = quat[3]

		#odom = Odometry()


	def __init__(self):

		self.base_link()
		rospy.Subscriber("joy", Joy, self.action)
		#rospy.Subscriber("odometry/filtered", Odometry, self.base_link)

		rate = rospy.Rate(20)
		self.start()

		while not rospy.is_shutdown():

			br = tf2_ros.TransformBroadcaster()
			t = geometry_msgs.msg.TransformStamped()

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

			t.transform.translation.x = self.subposx
			t.transform.translation.y = self.subposy
			t.transform.translation.z = self.subposz
			t.transform.rotation.x = self.subrotx
			t.transform.rotation.y = self.subroty
			t.transform.rotation.z = self.subrotz
			t.transform.rotation.w = self.subrotw

			#odom = Odometry()

			t.header.stamp = rospy.Time.now()
			t.header.frame_id = "map"
			t.child_frame_id = "odom"
			br.sendTransform(t)	

			rate.sleep()

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