#!/usr/bin/env python

import rospy
import numpy as np
import sys
from sub8_msgs.msg import Thrust
from geometry_msgs.msg import WrenchStamped, Twist, PoseStamped
from orientation_library import transformations as trns
from orientation_library import oritools as ori
from nav_msgs.msg import Odometry

class control_sub():

	def PD(self, data):
		kp = np.array([.9, .9, .9])  # proportional gain (body frame roll, pitch, yaw)
		kd = np.array([1.1, 1.1, 1.1])  # derivative gain (body frame rolling, pitching, yawing)

		self.qW = np.array([data.pose.pose.orientation.w, data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z]) 
		self.w = np.array([data.twist.twist.angular.x, data.twist.twist.angular.y, data.twist.twist.angular.z])

		self.pW = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])

		self.p_desW = np.array([0.0, 0.0, 0.75])
		
		self.q_desW = np.array([1.0, 0.0, 0.0, 0.0])
		self.w_des = np.array([0, 0, 0])

		self.qtrns = trns.quaternion_matrix(self.qW)

		#rospy.logwarn(self.qW)

		self.q9 = self.qtrns[:3,:3]
		
		#rospy.logwarn(self.qW)
		
		self.qT = np.transpose(self.q9)
		
		#rospy.logwarn(self.qW)

		self.w_err = self.w_des - self.w



		#self.q_err = ori.error(self.q, self.q_des)
		self.q_err = np.dot(ori.error(self.qW, self.q_desW),self.qT)

		#rospy.logwarn(self.q_err)

		#kdW = np.diag(ori.qapply_matrix(q, np.diag(kd)))

		torque_amnt = (kd * self.w_err) + (kp * self.q_err)
		#print torque

		wrench = WrenchStamped()

		wrench.header.frame_id = "/base_link"
		wrench.wrench.torque.x = torque_amnt[0]
		wrench.wrench.torque.y = torque_amnt[1]
		wrench.wrench.torque.z = torque_amnt[2]

		print wrench.wrench.torque.x

		self.thruster.publish(wrench)
		
		#print self.w_err
		

	def __init__(self):
		rospy.Subscriber("/odometry/filtered", Odometry, self.PD)
		self.thruster = rospy.Publisher("/wrench", WrenchStamped, queue_size=1)



def main(args):
	rospy.init_node('pd_controller', anonymous=True)

	control_sub()

        try:
		rospy.spin()
        except rospy.ROSInterruptException:
		print "Shutting down"
                pass
	


if __name__ == '__main__':
	main(sys.argv)
