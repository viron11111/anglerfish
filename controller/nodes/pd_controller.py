#!/usr/bin/env python

import rospy
import numpy as np
import sys
from sub8_msgs.msg import Thrust
import geometry_msgs.msg 
from geometry_msgs.msg import WrenchStamped, Twist, PoseStamped
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from orientation_library import transformations as trns
from orientation_library import oritools as ori
from nav_msgs.msg import Odometry
import tf, tf2_ros

from dynamic_reconfigure.server import Server
from controller.cfg import GainsConfig

class control_sub():

	def rc_pos(self, data):

		self.p_desW = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
		self.q_desW = np.array([data.pose.pose.orientation.w, data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z])

		br = tf2_ros.TransformBroadcaster()
		t = geometry_msgs.msg.TransformStamped()

		t.header.stamp = rospy.Time.now()
		t.header.frame_id = "odom"
		t.child_frame_id = "directed"
		t.transform.translation.x = self.p_desW[0]
		t.transform.translation.y = self.p_desW[1]
		t.transform.translation.z = self.p_desW[2]
		t.transform.rotation.w = self.q_desW[0]
		t.transform.rotation.x = self.q_desW[1]
		t.transform.rotation.y = self.q_desW[2]
		t.transform.rotation.z = self.q_desW[3]
		br.sendTransform(t)

	def PD(self, data):
		self.qW = np.array([data.pose.pose.orientation.w, data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z]) 
		self.w = np.array([data.twist.twist.angular.x, data.twist.twist.angular.y, data.twist.twist.angular.z])

		self.pW = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z]) #current position
		self.linear_acceleration = np.array([data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z])
		
		self.w_des = np.array([0.0, 0.0, 0.0]) #dersired angular twist once at desired orientation

		self.p_des = np.array([0.0, 0.0, 0.0]) #desired acceleration once at position 

		self.qtrns = trns.quaternion_matrix(self.qW)

		self.q9 = self.qtrns[:3,:3]

		self.qT = np.transpose(self.q9)

		self.w_err = self.w_des - self.w   #ORIENTATION angular twist error for derivative part of PID controller

		self.pw_err = self.p_des - self.linear_acceleration #POSITION linear acceleration error for kd (PD controller)

		self.p_err = np.dot(self.qT, (self.p_desW - self.pW))  #POSITION error for proportional part of PD controller

		self.q_err = np.dot(self.qT, ori.error(self.qW, self.q_desW))  #ORIENTATION error for proportional part of PID controller

		#                  Proportional                Derivative                Integral
		torque_amnt = (self.t_kp * self.q_err) + (self.t_kd * self.w_err) + (self.t_ki * self.i_err) #PID equation for torque (orientation)

		self.i_err = np.array(self.i_err + self.q_err)  #ORIENTATION integrator error

		np.clip(self.i_err, -5, 5, self.i_err) #ORIENTATION clipping integrate error to -5 or to 5 (prevent "run-off")

		#                  Proportional                 Derivative                 Integral
		force_amnt = (self.f_kp * self.p_err) - (self.f_kd * self.pw_err) + (self.f_ki * self.fi_err) #PID equation for force (position)

		self.fi_err = np.array(self.fi_err + self.p_err)

		np.clip(self.fi_err, -5, 5, self.fi_err) #POSITIO clipping integrate error to -5 or to 5 (prevent "run-off")

		wrench = WrenchStamped()

		wrench.header.frame_id = "/base_link"
		wrench.wrench.torque.x = torque_amnt[0]
		wrench.wrench.torque.y = torque_amnt[1]
		wrench.wrench.torque.z = torque_amnt[2]

		wrench.wrench.force.x = force_amnt[0]
		wrench.wrench.force.y = force_amnt[1]
		wrench.wrench.force.z = force_amnt[2]

		self.thruster.publish(wrench)

	def callback_gains(self, config, level):
		self.t_kp[0] = "{t_kp_x}".format(**config)
		self.t_kp[1] = "{t_kp_y}".format(**config)
		self.t_kp[2] = "{t_kp_z}".format(**config)

		self.t_kd[0] = "{t_kd_x}".format(**config)
		self.t_kd[1] = "{t_kd_y}".format(**config)
		self.t_kd[2] = "{t_kd_z}".format(**config)

		self.t_ki[0] = "{t_ki_x}".format(**config)
		self.t_ki[1] = "{t_ki_y}".format(**config)
		self.t_ki[2] = "{t_ki_z}".format(**config)

		self.f_kp[0] = "{f_kp_x}".format(**config)
		self.f_kp[1] = "{f_kp_y}".format(**config)
		self.f_kp[2] = "{f_kp_z}".format(**config)

		self.f_ki[0] = "{f_ki_x}".format(**config)
		self.f_ki[1] = "{f_ki_y}".format(**config)
		self.f_ki[2] = "{f_ki_z}".format(**config)		

		self.f_kd[0] = "{f_kd_x}".format(**config)
		self.f_kd[1] = "{f_kd_y}".format(**config)
		self.f_kd[2] = "{f_kd_z}".format(**config)

		return config           

	def __init__(self):
		rospy.Subscriber("/odometry/filtered", Odometry, self.PD)
		rospy.Subscriber("RC_position", PoseWithCovarianceStamped, self.rc_pos)
		self.thruster = rospy.Publisher("/wrench", WrenchStamped, queue_size=1)
		rospy.Subscriber

		self.t_kp = np.array([0.0, 0.0, 0.0])  # proportional gain (body frame roll, pitch, yaw)
		self.t_ki = np.array([0.0, 0.0, 0.0])
		self.t_kd = np.array([0.0, 0.0, 0.0])  # derivative gain (body frame rolling, pitching, yawing)

		self.f_kp = np.array([0.0, 0.0, 0.0])
		self.f_ki = np.array([0.0, 0.0, 0.0])
		self.f_kd = np.array([0.0, 0.0, 0.0])

		self.i_err = np.array([0.0, 0.0, 0.0])  #integrator error for orientation
		self.fi_err = np.array([0.0, 0.0, 0.0]) #integrator error for position

		self.p_desW = np.array([0.0, 0.0, -0.5])
		self.q_desW = np.array([1.0, 0.0, 0.0, 0.0])


		srv = Server(GainsConfig, self.callback_gains)

		r = rospy.Rate(30)

		while not rospy.is_shutdown():
				r.sleep()

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
