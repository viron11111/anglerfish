#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
import geometry_msgs.msg
import numpy as np
import tf, tf2_ros
import math
from sensor_msgs.msg import Imu
import time

import csv
from time import localtime,strftime
import datetime

class integration():

	def create_files(self):
		self.start_time = 0
		self.time_diff = 0

		date_time = strftime("%d_%m_%y_%H_%M_%S", localtime())
		file_name1 = "/home/andy/catkin_ws/src/anglerfish/integrate_accel/data/razor_%s.csv" % date_time
		
		self.velocity_file = csv.writer(open(file_name1,'w'))
		self.velocity_file.writerow(["Time", "Velocity", "Position"])

	def linear_accel(self, data):
		self.new_time = rospy.get_rostime()

		self.x_accel = data.linear_acceleration.x
		self.y_accel = data.linear_acceleration.y
		self.z_accel = data.linear_acceleration.z

		self.time_diff = self.new_time - self.old_time
		self.sum_time = self.time_diff.secs + self.time_diff.nsecs/1000000000.0

		time = self.new_time - self.start_time
		run_time = time.secs + time.nsecs/1000000000.0

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
		
		self.y_grav = math.cos(self.pitch)*math.sin(self.roll)*-9.80665
		self.x_grav = math.sin(self.pitch)*-9.80665
		self.z_grav = -math.cos(self.pitch)*math.cos(self.roll)*-9.80665

		#rospy.loginfo(self.x_grav)
		#ime.sleep(1)

		#linear acceleration
		#FIR low pass filter
		measfir1 = self.x_accel
		self.measured = measfir1*0.5 + self.measfir2*0.5 # +self.measfir3*0.25 + self.measfir4*0.25# + self.measfir5*0.2
		#self.measfir5 = self.measfir4
		#self.measfir4 = self.measfir3
		#self.measfir3 = self.measfir2
		self.measfir2 = measfir1

		#FIR high pass filter
		#measfir1 = self.x_accel
		#self.measured = self.measfir2 - 2*(measfir1*0.5 - self.measfir2*0.5)
		#self.measfir2 = measfir1

		#Estimate gravity experience
		#FIR low pass filter
		'''orinfir1 = self.x_grav
		orientation = orinfir1*0.5 + self.orinfir2*0.5
		self.orinfir2 = orinfir1'''	

		#Estimate gravity experience
		#FIR high pass filter
		orinfir1 = self.x_grav
		self.orientation = self.orinfir2 - 2*(orinfir1*0.5 - self.orinfir2*0.5)
		self.orinfir2 = orinfir1



		acceleration = self.x_accel - self.x_grav #self.orientation #(measured - orientation) #x_pos + vel

		velocity = acceleration*self.sum_time#acceleration*self.sum_time

		self.velfir1 = velocity

		'''if self.holder3 == 0.0:
			rospy.loginfo("zero")
			self.velfiri1 = 0.0
			time.sleep(1)'''


		self.speed = self.speed + velocity #self.velfir2 - 2*(self.velfir1*0.5 - self.velfir2*0.5)


		#self.velfir2 = self.velfir1

		#rospy.loginfo(self.speed)
		#time.sleep(1)

		self.vel_new = velocity
		velocity_print = (self.vel_old - self.vel_new)/self.sum_time
		#rospy.loginfo(velocity)
		self.vel_old = self.vel_new

		self.position = self.position + self.speed*self.sum_time

		self.pospub.header.stamp = rospy.Time.now()
		self.pospub.header.frame_id = 'desired_position' # i.e. '/odom'

		self.pospub.pose.pose.position.x = self.speed #estimated_gravity
		self.pospub.pose.pose.position.y = self.position #linear Acceleration
		self.pospub.pose.pose.position.z = velocity
		self.pospub.pose.pose.orientation.x = self.subrotx
		self.pospub.pose.pose.orientation.y = self.subroty
		self.pospub.pose.pose.orientation.z = self.subrotz
		self.pospub.pose.pose.orientation.w = self.subrotw
		#self.pospub.pose.covariance=(np.eye(6)*.001).flatten()

		self.pose_pub.publish(self.pospub)
		self.old_time = rospy.get_rostime()

		self.velocity_file.writerow([run_time, self.speed, self.position])


		#rospy.loginfo(run_time)


	def orientation(self, data):

		self.qW = np.array([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]) 
		self.w = np.array([data.twist.twist.angular.x, data.twist.twist.angular.y, data.twist.twist.angular.z])

		self.new_time = rospy.get_rostime()

		self.x_accel = data.twist.twist.linear.x
		self.y_accel = data.twist.twist.linear.y
		self.z_accel = data.twist.twist.linear.z

		self.time_diff = self.new_time - self.old_time
		self.sum_time = self.time_diff.secs + self.time_diff.nsecs/1000000000.0

		time = self.new_time - self.start_time
		run_time = time.secs + time.nsecs/1000000000.0

		quat = (
			data.pose.pose.orientation.x,
			data.pose.pose.orientation.y,
			data.pose.pose.orientation.z,
			data.pose.pose.orientation.w)

		euler = tf.transformations.euler_from_quaternion(quat)

		self.roll = euler[0]
		self.pitch = euler[1]
		self.yaw = euler[2]

		self.subrotx = 0.0 # quat[0]
		self.subroty = 0.0 # quat[1]
		self.subrotz = 0.0 # quat[2]
		self.subrotw = 0.0 # quat[3]
		
		self.y_grav = math.cos(self.pitch)*math.sin(self.roll)*-9.80665
		self.x_grav = math.sin(self.pitch)*-9.80665
		self.z_grav = -math.cos(self.pitch)*math.cos(self.roll)*-9.80665

		#rospy.loginfo(self.x_grav)
		#ime.sleep(1)

		#linear acceleration
		#FIR low pass filter
		measfir1 = self.x_accel
		self.measured = measfir1*0.5 + self.measfir2*0.5 # +self.measfir3*0.25 + self.measfir4*0.25# + self.measfir5*0.2
		#self.measfir5 = self.measfir4
		#self.measfir4 = self.measfir3
		#self.measfir3 = self.measfir2
		self.measfir2 = measfir1

		#FIR high pass filter
		#measfir1 = self.x_accel
		#self.measured = self.measfir2 - 2*(measfir1*0.5 - self.measfir2*0.5)
		#self.measfir2 = measfir1

		#Estimate gravity experience
		#FIR low pass filter
		'''orinfir1 = self.x_grav
		orientation = orinfir1*0.5 + self.orinfir2*0.5
		self.orinfir2 = orinfir1'''	

		#Estimate gravity experience
		#FIR high pass filter
		orinfir1 = self.x_grav
		self.orientation = self.orinfir2 - 2*(orinfir1*0.5 - self.orinfir2*0.5)
		self.orinfir2 = orinfir1



		acceleration = self.x_accel - self.x_grav #self.orientation #(measured - orientation) #x_pos + vel

		velocity = acceleration*self.sum_time#acceleration*self.sum_time

		self.velfir1 = velocity

		'''if self.holder3 == 0.0:
			rospy.loginfo("zero")
			self.velfiri1 = 0.0
			time.sleep(1)'''


		self.speed = self.speed + velocity #self.velfir2 - 2*(self.velfir1*0.5 - self.velfir2*0.5)


		#self.velfir2 = self.velfir1

		#rospy.loginfo(self.speed)
		#time.sleep(1)

		self.vel_new = velocity
		velocity_print = (self.vel_old - self.vel_new)/self.sum_time
		#rospy.loginfo(velocity)
		self.vel_old = self.vel_new

		self.position = self.position + self.speed*self.sum_time

		self.pospub.header.stamp = rospy.Time.now()
		self.pospub.header.frame_id = 'desired_position' # i.e. '/odom'

		self.pospub.pose.pose.position.x = self.speed #estimated_gravity
		self.pospub.pose.pose.position.y = self.position #linear Acceleration
		self.pospub.pose.pose.position.z = velocity
		self.pospub.pose.pose.orientation.x = self.subrotx
		self.pospub.pose.pose.orientation.y = self.subroty
		self.pospub.pose.pose.orientation.z = self.subrotz
		self.pospub.pose.pose.orientation.w = self.subrotw
		#self.pospub.pose.covariance=(np.eye(6)*.001).flatten()

		self.pose_pub.publish(self.pospub)
		self.old_time = rospy.get_rostime()

		self.velocity_file.writerow([run_time, self.speed, self.position])


		#rospy.loginfo(run_time)		


	def __init__(self):

		self.init_vars = 0
		self.init_vars2 = 0

		self.create_files()

		#rospy.Subscriber("/odometry/filtered", Odometry, self.orientation)
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
		self.start_time = rospy.get_rostime()

		self.accel_flag = 0
		self.calculated_flag = 0

		self.subrotx = 0.0
		self.subroty = 0.0
		self.subrotz = 0.0
		self.subrotw = 0.0

		self.roll = 0.0
		self.pitch = 0.0
		self.pitch2 = 0.0
		self.yaw = 0.0

		self.speed = 0.0
		self.measfir2 = 0.0
		self.measfir3 = 0.0
		self.measfir4 = 0.0
		self.measfir5 = 0.0
		self.orinfir2 = 0.0

		self.velfir1 = 0.0
		self.velfir2 = 0.0

		self.holder1 = 0.0
		self.holder2 = 0.0
		self.holder3 = 0.0
		self.holder4 = 0.0
		self.holder5 = 0.0
		self.holder6 = 0.0
		self.holder7 = 0.0
		self.holder8 = 0.0
		self.holder9 = 0.0

		self.pos_old = 0.0
		self.pos_new = 0.0

		self.vel_old = 0.0
		self.vel_new = 0.0

		self.position = 0.0

		r = rospy.Rate(60)

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