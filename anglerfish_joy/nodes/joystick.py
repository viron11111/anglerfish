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

		self.movex = 0.0
		self.movey = 0.0
		self.yaw = 0.0

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
		elif data.buttons[3] == 1:
			rc = PoseWithCovarianceStamped()

			rc.header.stamp = rospy.Time.now()
			rc.header.frame_id = 'desired_position' # i.e. '/odom'

			rc.pose.pose.position.x = self.subposx
			rc.pose.pose.position.y = self.subposy
			rc.pose.pose.position.z = self.subposz
			rc.pose.pose.orientation.x = self.subrotx
			rc.pose.pose.orientation.y = self.subroty
			rc.pose.pose.orientation.z = self.subrotz
			rc.pose.pose.orientation.w = self.subrotw
			#rc.pose.covariance=(np.eye(6)*.001).flatten()

			self.pose_pub.publish(rc)
			#rospy.logwarn(rc)


		if data.axes[1] < 0.015 and data.axes[1] > -0.015:
			self.movex = 0.0
		elif data.axes[1] != 0.0:
			self.movex = data.axes[1]*.05

		if data.axes[0] < 0.015 and data.axes[0] > -0.015:
			self.movey = 0.0
		elif data.axes[0] != 0.0:
			self.movey = data.axes[0]*.05

		if data.axes[3] < 0.015 and data.axes[3] > -0.015:
			self.yaw = 0.0
		if data.axes[3] != 0.0:
			self.yaw = data.axes[3]*0.1






		#odom = Odometry()


	def __init__(self):

		self.base_link()
		rospy.Subscriber("joy", Joy, self.action)
		self.pose_pub = rospy.Publisher("RC_position", PoseWithCovarianceStamped, queue_size = 1)



		rate = rospy.Rate(20)
		self.start()

		while not rospy.is_shutdown():

			br = tf2_ros.TransformBroadcaster()
			t = geometry_msgs.msg.TransformStamped()

			self.subposx += self.movex
			self.subposy += self.movey

			quat = (
				self.subrotx,
				self.subroty,
				self.subrotz,
				self.subrotw)

			euler = tf.transformations.euler_from_quaternion(quat)
			
			turn = euler[2] + self.yaw
			euler = (
				euler[0],
				euler[1],
				turn)
			#rospy.loginfo(euler)

			quat = tf.transformations.quaternion_from_euler(euler[0],euler[1], euler[2])
			self.subrotx = quat[0]
			self.subroty = quat[1]
			self.subrotz = quat[2]
			self.subrotw = quat[3]

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

			t.transform.translation.x = 0
			t.transform.translation.y = 0
			t.transform.translation.z = 0
			t.transform.rotation.x = 0
			t.transform.rotation.y = 0
			t.transform.rotation.z = 0
			t.transform.rotation.w = 1.0

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