#!/usr/bin/python

import rospy
import math
import geometry_msgs.msg 
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from sensor_msgs.msg import Joy
import tf, tf2_ros
from nav_msgs.msg import Odometry
from anglerfish_joy.msg import rpy, move

class mission(object):

	def rc_pos(self, data):

		self.x_pos = data.pose.pose.position.x
		self.y_pos = data.pose.pose.position.y
		self.z_pos = data.pose.pose.position.z

	def __init__(self):

		mission_counter = 0
		counter = 0
		time_per_move = 200

		self.roll = 0
		self.pitch = 0
		self.yaw = 0
		self.X = 0
		self.Y = 0
		#self.Z = 0

		self.x_pos = 0
		self.y_pos = 0
		self.z_pos = 0	

		self.move_rate = 0.02

		desired_x = 0
		desired_y = -1


		self.orientation = rospy.Publisher("/rpy", rpy, queue_size = 1)

		self.position = rospy.Publisher("/mission_move_rov", PoseWithCovarianceStamped, queue_size = 1)
		rospy.Subscriber("/RC_position", PoseWithCovarianceStamped, self.rc_pos)
		
		rate = rospy.Rate(10)

		rospy.wait_for_message('/RC_position', PoseWithCovarianceStamped)
		rospy.loginfo(self.x_pos)

		#while (self.X == 0 and self.Y == 0):
		#	self.X = self.x_pos
		#	self.Y = self.y_pos

		while not rospy.is_shutdown():

			if self.x_pos > desired_x:
				self.X = self.x_pos - self.move_rate
			elif self.x_pos < desired_x:
				self.X = self.x_pos + self.move_rate

			if self.y_pos > desired_y:
				self.Y = self.y_pos - self.move_rate
			elif self.y_pos < desired_y:
				self.Y = self.y_pos + self.move_rate
				
			if (self.x_pos <= desired_x + 0.02 and self.x_pos >= desired_x - 0.02 and self.y_pos <= desired_y + 0.02 and self.y_pos >= desired_y - 0.02):
				self.Y = desired_y
				self.X = desired_x
				rospy.loginfo("mission complete")
				rospy.signal_shutdown("mission complete")

			'''o = rpy()
			p = move()

			o.header.stamp = rospy.Time.now()
			o.header.frame_id = 'desired_orientation' # i.e. '/odom'

			o.Roll = self.roll
			o.Pitch = self.pitch
			o.Yaw = self.yaw

			p.header.stamp = rospy.Time.now()
			p.header.frame_id = 'desired_position' # i.e. '/odom'

			p.X = self.X
			p.Y = self.Y
			p.Z = self.Z

			self.orientation.publish(o)
			self.position.publish(p)'''


			rc = PoseWithCovarianceStamped()

			rc.header.stamp = rospy.Time.now()
			rc.header.frame_id = 'auto_desired_position' # i.e. '/odom'

			rc.pose.pose.position.x = self.X
			rc.pose.pose.position.y = self.Y
			rc.pose.pose.position.z = self.z_pos
			rc.pose.pose.orientation.x = 0		
			rc.pose.pose.orientation.y = 0
			rc.pose.pose.orientation.z = 0
			rc.pose.pose.orientation.w = 1.0

			self.position.publish(rc)

			rate.sleep()

def main():
	rospy.init_node('mission', anonymous=False)

	mission()

	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		print "Shutting down"
		pass


if __name__ == '__main__':
	main() #sys.argv
