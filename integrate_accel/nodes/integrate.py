#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
import geometry_msgs.msg
import numpy as np
import tf, tf2_ros
import math
from sensor_msgs.msg import Imu

class integration():

	def linear_accel(self, data):

		self.x_accel = data.linear_acceleration.x
		self.y_accel = data.linear_acceleration.y
		self.z_accel = data.linear_acceleration.z

		if self.init_vars == 0:
			holder = 0.0
			x_vel_old = 0.0
			x_vel = 0.0
			y_vel = 0.0
			z_vel = 0.0
			x_accel_diff_old = 0.0
			x_pos = 0.0
			y_pos = 0.0
			z_pos = 0.0
			fir1 = 0
			fir2 = 0
			fir3 = 0
			fir4 = 0.0
			measfir1 = 0.0
			self.measfir2 = 0.0
			self.measfir3 = 0.0
			self.measfir4 = 0.0
			self.measfir5 = 0.0
			orinfir1 = 0.0
			self.orinfir2 = 0.0
			orinfir3 = 0.0
			orinfir4 = 0.0
			orinfir5 = 0.0

			self.accn1 = 0.0
			self.accn2 = 0.0
			self.accy0 = data.linear_acceleration.x

			velfir1 = 0.0
			velfir2 = 0.0
			accfir1 = 0.0
			accfir2 = 0.0
			vel = 0.0
			posfir1 = 0.0
			posfir2 = 0.0
			pos = 0.0
			diff1 = 0.0
			diff2 = 0.0
			self.position = 0.0
			self.orientation = 0.0 #self.x_accel
			self.vx = self.x_accel

			self.init_vars = 1



		self.new_time = rospy.get_rostime()
		self.time_diff = self.new_time - self.old_time
		self.sum_time = self.time_diff.secs + self.time_diff.nsecs/1000000000.0

		quat = (
			data.orientation.x,
			data.orientation.y,
			data.orientation.z,
			data.orientation.w)

		euler = tf.transformations.euler_from_quaternion(quat)

		self.roll = euler[0]
		self.pitch = euler[1]
		self.yaw = euler[2]

		self.subrotx = 0.0 # quat[0]
		self.subroty = 0.0 # quat[1]
		self.subrotz = 0.0 # quat[2]
		self.subrotw = 0.0 # quat[3]
		
		self.y_grav = -math.cos(self.pitch)*math.sin(self.roll)*-9.80665
		self.x_grav = math.sin(self.pitch)*-9.80665
		self.z_grav = -math.cos(self.pitch)*math.cos(self.roll)*-9.80665

		#linear acceleration
		#FIR low pass filter
		measfir1 = self.x_accel
		measured = measfir1*0.2 + self.measfir2*0.2 +self.measfir3*0.2 + self.measfir4*0.2 + self.measfir5*0.2
		self.measfir5 = self.measfir4
		self.measfir4 = self.measfir3
		self.measfir3 = self.measfir2
		self.measfir2 = measfir1

		#FIR high pass filter
		'''measfir1 = self.x_accel
		measured = 2*(measfir1*0.5 - self.measfir2*0.5)
		self.measfir2 = measfir1'''

		#Estimate gravity experience
		#FIR low pass filter
		orinfir1 = self.x_grav
		orientation = orinfir1*0.5 + self.orinfir2*0.5
		self.orinfir2 = orinfir1	

		#Estimate gravity experience
		#FIR high pass filter
		'''orinfir1 = self.x_grav
		self.orientation = self.orientation + (orinfir1*0.5 - self.orinfir2*0.5)
		self.orinfir2 = orinfir1'''

		acceleration = measured- orientation #(measured - orientation) #x_pos + vel

		velocity = acceleration*self.sum_time#acceleration*self.sum_time

		self.speed = self.speed + velocity

		self.position = self.position + self.speed*self.sum_time

		self.pospub.header.stamp = rospy.Time.now()
		self.pospub.header.frame_id = 'desired_position' # i.e. '/odom'

		self.pospub.pose.pose.position.x = orientation #estimated_gravity
		self.pospub.pose.pose.position.y = measured #linear Acceleration
		self.pospub.pose.pose.position.z = velocity
		self.pospub.pose.pose.orientation.x = self.subrotx
		self.pospub.pose.pose.orientation.y = self.subroty
		self.pospub.pose.pose.orientation.z = self.subrotz
		self.pospub.pose.pose.orientation.w = self.subrotw
		#self.pospub.pose.covariance=(np.eye(6)*.001).flatten()

		self.pose_pub.publish(self.pospub)
		self.old_time = rospy.get_rostime()

	def orientation(self, data):
		if self.calculated_flag == 0:
			self.qW = np.array([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]) 
			self.w = np.array([data.twist.twist.angular.x, data.twist.twist.angular.y, data.twist.twist.angular.z])

			#this section converts quaternions to euler, applys yaw, then converts euler back to quaternion
			'''quat = (
				self.qW[0],
				self.qW[1],
				self.qW[2],
				self.qW[3])

			euler = tf.transformations.euler_from_quaternion(quat)

			roll = euler[0]
			pitch = euler[1]
			yaw = euler[2]

			quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
			self.subrotx = quat[0]
			self.subroty = quat[1]
			self.subrotz = quat[2]
			self.subrotw = quat[3]
			
			self.y_grav = -math.cos(pitch)*math.sin(roll)*-9.80665
			self.x_grav = math.sin(pitch)*-9.80665
			self.z_grav = -math.cos(pitch)*math.cos(roll)*-9.80665
			self.calculated_flag = 1'''


	def __init__(self):

		self.init_vars = 0
		self.init_vars2 = 0

		rospy.Subscriber("/odometry/filtered", Odometry, self.orientation)
		rospy.Subscriber("/imu/data", Imu, self.linear_accel)

		self.pose_pub = rospy.Publisher("integrated_position", PoseWithCovarianceStamped, queue_size = 1)
		self.pospub = PoseWithCovarianceStamped()

		self.old_time = rospy.get_rostime()
		#self.thruster = rospy.Publisher("/wrench", WrenchStamped, queue_size=1)


		#srv = Server(GainsConfig, self.callback_gains)

		self.x_accel = 0.0
		self.y_accel = 0.0
		self.z_accel = 0.0
		self.y_grav = 0.0
		self.x_grav = 0.0
		self.z_grav = 0.0
		self.time_diff = 0.0
		self.sum_time = 0.0
		self.accel_flag = 0
		self.calculated_flag = 0

		self.subrotx = 0.0
		self.subroty = 0.0
		self.subrotz = 0.0
		self.subrotw = 0.0

		self.roll = 0.0
		self.pitch = 0.0
		self.yaw = 0.0

		self.speed = 0.0

		r = rospy.Rate(20)

		while not rospy.is_shutdown():
			r.sleep()

def main():
	rospy.init_node('integrate_accelerometers', anonymous=True)

	integration()

        try:
		rospy.spin()
        except rospy.ROSInterruptException:
		print "Shutting down"
                pass
	


if __name__ == '__main__':
	main()