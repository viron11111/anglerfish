#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
import numpy as np
import tf, tf2_ros
import math
from sensor_msgs.msg import Imu

class integration():

	def linear_accel(self, data):
		if self.accel_flag == 0:

			self.x_accel = data.linear_acceleration.x
			self.y_accel = data.linear_acceleration.y
			self.z_accel = data.linear_acceleration.z
			self.accel_flag = 1

		#rospy.loginfo(data.linear_acceleration.x)

	def orientation(self, data):
		if self.calculated_flag == 0:
			self.qW = np.array([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]) 
			self.w = np.array([data.twist.twist.angular.x, data.twist.twist.angular.y, data.twist.twist.angular.z])

			#this section converts quaternions to euler, applys yaw, then converts euler back to quaternion
			quat = (
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
			self.calculated_flag = 1


	def __init__(self):
		rospy.Subscriber("/odometry/filtered", Odometry, self.orientation)
		rospy.Subscriber("/imu/data", Imu, self.linear_accel)

		self.pose_pub = rospy.Publisher("integrated_position", PoseWithCovarianceStamped, queue_size = 1)
		pospub = PoseWithCovarianceStamped()

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
		measfir2 = 0.0
		measfir3 = 0.0
		measfir4 = 0.0
		measfir5 = 0.0

		orinfir1 = 0.0
		orinfir2 = 0.0
		orinfir3 = 0.0
		orinfir4 = 0.0
		orinfir5 = 0.0

		accfir1 = 0.0
		accfir2 = 0.0

		velfir1 = 0.0
		velfir2 = 0.0

		vel = 0.0

		posfir1 = 0.0
		posfir2 = 0.0

		pos = 0.0

		diff1 = 0.0
		diff2 = 0.0

		r = rospy.Rate(50)

		while not rospy.is_shutdown():

			if self.accel_flag == 1 and self.calculated_flag == 1:

				self.new_time = rospy.get_rostime()
				self.time_diff = self.new_time - self.old_time
				self.sum_time = self.time_diff.secs + self.time_diff.nsecs/1000000000.0

				#FIR low pass filter
				measfir1 = self.x_accel
				measured = measfir1*0.2 + measfir2*0.2 + measfir3*0.2 + measfir4*0.2 + measfir5*0.2
				measfir5 = measfir4
				measfir4 = measfir3
				measfir3 = measfir2
				measfir2 = measfir1

				#FIR low pass filter
				orinfir1 = self.x_grav
				orientation = orinfir1*0.2 + orinfir2*0.2 + orinfir3*0.2 + orinfir4*0.2 + orinfir5*0.2
				orinfir5 = orinfir4
				orinfir4 = orinfir3
				orinfir3 = orinfir2
				orinfir2 = orinfir1		

				#x_pos = measured #x_pos + vel
				y_pos = orientation


				#acceleration_based_on_orientation - measured_acceleration

				x_accel_diff = measured - orientation #self.x_grav - self.x_accel
				y_accel_diff = self.y_grav - self.y_accel
				z_accel_diff = self.z_grav - self.z_accel


				x_pos = x_accel_diff

				#FIR low pass filter
				fir1 = x_accel_diff
				acceleration = fir1*.5+ fir2*0.5
				#fir3 = fir2
				fir2 = fir1

				

				velfir1 = x_accel_diff*self.sum_time
				vel = vel + velfir1 #2*(velfir1*0.5 - velfir2*0.5)
				velfir2 = velfir1

				


				pos = pos + vel*self.sum_time

				holder = self.sum_time 

				pospub.header.stamp = rospy.Time.now()
				pospub.header.frame_id = 'desired_position' # i.e. '/odom'

				pospub.pose.pose.position.x = x_pos
				pospub.pose.pose.position.y = y_pos
				pospub.pose.pose.position.z = z_pos
				pospub.pose.pose.orientation.x = self.subrotx
				pospub.pose.pose.orientation.y = self.subroty
				pospub.pose.pose.orientation.z = self.subrotz
				pospub.pose.pose.orientation.w = self.subrotw
				#pospub.pose.covariance=(np.eye(6)*.001).flatten()

				self.pose_pub.publish(pospub)
				self.old_time = rospy.get_rostime()

				#rospy.loginfo(x_pos)
				#rospy.loginfo("time_diff: %i" % (self.time_diff.nsecs))
				#rospy.loginfo("linear_accel: %f estimated_accel: %f" % (self.x_accel, self.x_grav))
				#rospy.loginfo("diff: %f" % (diff))
				#rospy.loginfo(self.sum_time)

				self.accel_flag = 0
				self.calculated_flag = 0

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