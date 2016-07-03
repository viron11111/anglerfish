#!/usr/bin/python

import sys
import time
from time import localtime,strftime
import datetime
import math
import rospy
import csv
from std_msgs.msg import Float32
from std_msgs.msg import Header
from std_msgs.msg import String
from t100_thruster.msg import t100_thruster_feedback

from geometry_msgs.msg import Vector3
from sensor_msgs.msg import MagneticField
from orientation_estimater.msg import rpy_msg
from hmc5883l.msg import mag_raw
from mag_calibration.msg import mag_values

class start_test():

	def arm_thrusters(self):
		rospy.loginfo("Arming thrusters...")
		for i in range (0,10):
			self.thrust1_pub.publish(0.0)
			self.thrust2_pub.publish(0.0)
			self.thrust3_pub.publish(0.0)
			self.thrust4_pub.publish(0.0)
			self.thrust5_pub.publish(0.0)
			self.thrust6_pub.publish(0.0)
			self.rate.sleep()


	def forward(self):
		output = (float(self.counter)/self.resolution) * self.forward_thrust_force
		
		if output == 0:
			self.initial_x_no_comp = self.x_raw#-self.x_average
			self.initial_y_no_comp = self.y_raw#-self.y_average
			self.initial_z_no_comp = self.z_raw#-self.z_average
			self.initial_x_comp    = self.x_comp#-self.x_average
			self.initial_y_comp    = self.y_comp#-self.y_average
			self.initial_z_comp    = self.z_comp#-self.z_average

			self.initial_roll_raw  = self.roll_raw#-self.x_average
			self.initial_pitch_raw = self.pitch_raw#-self.y_average
			self.initial_yaw_raw   = self.yaw_raw#-self.z_average
			self.initial_roll_comp = self.roll_comp#-self.x_average
			self.initial_pitch_comp= self.pitch_comp#-self.y_average
			self.initial_yaw_comp  = self.yaw_comp#-self.z_average


		self.counter += 1

		if self.thruster == 1:
			self.thrust1_pub.publish(output)
			self.thruster1_file.writerow([output,self.x_raw-self.initial_x_no_comp, self.y_raw-self.initial_y_no_comp, self.z_raw-self.initial_z_no_comp, self.x_comp-self.initial_x_comp,self.y_comp-self.initial_y_comp,self.z_comp-self.initial_z_comp])
			self.thruster7_file.writerow([output,self.roll_raw-self.initial_roll_raw, self.pitch_raw-self.initial_pitch_raw,self.yaw_raw-self.initial_yaw_raw,self.roll_comp-self.initial_roll_comp,self.pitch_comp-self.initial_pitch_comp,self.yaw_comp-self.initial_yaw_comp])
		elif self.thruster == 2:
			self.thrust2_pub.publish(output)
			self.thruster2_file.writerow([output,self.x_raw-self.initial_x_no_comp, self.y_raw-self.initial_y_no_comp, self.z_raw-self.initial_z_no_comp])#, self.x_comp-self.initial_x_comp,self.y_comp-self.initial_y_comp,self.z_comp-self.initial_z_comp])
			self.thruster8_file.writerow([output,self.roll_raw-self.initial_roll_raw, self.pitch_raw-self.initial_pitch_raw,self.yaw_raw-self.initial_yaw_raw])#,self.roll_comp-self.initial_roll_comp,self.pitch_comp-self.initial_pitch_comp,self.yaw_comp-self.initial_yaw_comp])
		elif self.thruster == 3:
			self.thrust3_pub.publish(output)
			self.thruster3_file.writerow([output,self.x_raw-self.initial_x_no_comp, self.y_raw-self.initial_y_no_comp, self.z_raw-self.initial_z_no_comp])#, self.x_comp-self.initial_x_comp,self.y_comp-self.initial_y_comp,self.z_comp-self.initial_z_comp])
			self.thruster9_file.writerow([output,self.roll_raw-self.initial_roll_raw, self.pitch_raw-self.initial_pitch_raw,self.yaw_raw-self.initial_yaw_raw])#,self.roll_comp-self.initial_roll_comp,self.pitch_comp-self.initial_pitch_comp,self.yaw_comp-self.initial_yaw_comp])
		elif self.thruster == 4:
			self.thrust4_pub.publish(output)
			self.thruster4_file.writerow([output,self.x_raw-self.initial_x_no_comp, self.y_raw-self.initial_y_no_comp, self.z_raw-self.initial_z_no_comp])#, self.x_comp-self.initial_x_comp,self.y_comp-self.initial_y_comp,self.z_comp-self.initial_z_comp])
			self.thruster10_file.writerow([output,self.roll_raw-self.initial_roll_raw, self.pitch_raw-self.initial_pitch_raw,self.yaw_raw-self.initial_yaw_raw])#,self.roll_comp-self.initial_roll_comp,self.pitch_comp-self.initial_pitch_comp,self.yaw_comp-self.initial_yaw_comp])
		elif self.thruster == 5:
			self.thrust5_pub.publish(output)
			self.thruster5_file.writerow([output,self.x_raw-self.initial_x_no_comp, self.y_raw-self.initial_y_no_comp, self.z_raw-self.initial_z_no_comp])#, self.x_comp-self.initial_x_comp,self.y_comp-self.initial_y_comp,self.z_comp-self.initial_z_comp])
			self.thruster11_file.writerow([output,self.roll_raw-self.initial_roll_raw, self.pitch_raw-self.initial_pitch_raw,self.yaw_raw-self.initial_yaw_raw])#,self.roll_comp-self.initial_roll_comp,self.pitch_comp-self.initial_pitch_comp,self.yaw_comp-self.initial_yaw_comp])
		elif self.thruster == 6:
			self.thrust6_pub.publish(output)
			self.thruster6_file.writerow([output,self.x_raw-self.initial_x_no_comp, self.y_raw-self.initial_y_no_comp, self.z_raw-self.initial_z_no_comp, self.x_comp-self.initial_x_comp,self.y_comp-self.initial_y_comp,self.z_comp-self.initial_z_comp])
			self.thruster12_file.writerow([output,self.roll_raw-self.initial_roll_raw, self.pitch_raw-self.initial_pitch_raw,self.yaw_raw-self.initial_yaw_raw,self.roll_comp-self.initial_roll_comp,self.pitch_comp-self.initial_pitch_comp,self.yaw_comp-self.initial_yaw_comp])
		else:
			self.thrust1_pub.publish(0.0)
			self.thrust2_pub.publish(0.0)
			self.thrust3_pub.publish(0.0)
			self.thrust4_pub.publish(0.0)
			self.thrust5_pub.publish(0.0)
			self.thrust6_pub.publish(0.0)						

		if self.counter == self.resolution + 1:
			self.counter = 0
			self.thruster += 1

			self.thrust1_pub.publish(0.0)
			self.thrust2_pub.publish(0.0)
			self.thrust3_pub.publish(0.0)
			self.thrust4_pub.publish(0.0)
			self.thrust5_pub.publish(0.0)
			self.thrust6_pub.publish(0.0)

			time.sleep(3)

			self.arm_thrusters()

	def reverse(self):
		output = (float(self.counter)/self.resolution) * self.reverse_thrust_force

		if output == 0:
			self.initial_x_no_comp = self.x_raw#-self.x_average
			self.initial_y_no_comp = self.y_raw#-self.y_average
			self.initial_z_no_comp = self.z_raw#-self.z_average
			self.initial_x_comp    = self.x_comp#-self.x_average
			self.initial_y_comp    = self.y_comp#-self.y_average
			self.initial_z_comp    = self.z_comp#-self.z_average

			self.initial_roll_raw  = self.roll_raw#-self.x_average
			self.initial_pitch_raw = self.pitch_raw#-self.y_average
			self.initial_yaw_raw   = self.yaw_raw#-self.z_average
			self.initial_roll_comp = self.roll_comp#-self.x_average
			self.initial_pitch_comp= self.pitch_comp#-self.y_average
			self.initial_yaw_comp  = self.yaw_comp#-self.z_average

		self.counter += 1

		if self.thruster == 1:
			self.thrust1_pub.publish(output)
			self.thruster1_file.writerow([output,self.x_raw-self.initial_x_no_comp, self.y_raw-self.initial_y_no_comp, self.z_raw-self.initial_z_no_comp, self.x_comp-self.initial_x_comp,self.y_comp-self.initial_y_comp,self.z_comp-self.initial_z_comp])
			self.thruster7_file.writerow([output,self.roll_raw-self.initial_roll_raw, self.pitch_raw-self.initial_pitch_raw,self.yaw_raw-self.initial_yaw_raw,self.roll_comp-self.initial_roll_comp,self.pitch_comp-self.initial_pitch_comp,self.yaw_comp-self.initial_yaw_comp])
		elif self.thruster == 2:
			self.thrust2_pub.publish(output)
			self.thruster2_file.writerow([output,self.x_raw-self.initial_x_no_comp, self.y_raw-self.initial_y_no_comp, self.z_raw-self.initial_z_no_comp])#, self.x_comp-self.initial_x_comp,self.y_comp-self.initial_y_comp,self.z_comp-self.initial_z_comp])
			self.thruster8_file.writerow([output,self.roll_raw-self.initial_roll_raw, self.pitch_raw-self.initial_pitch_raw,self.yaw_raw-self.initial_yaw_raw])#,self.roll_comp-self.initial_roll_comp,self.pitch_comp-self.initial_pitch_comp,self.yaw_comp-self.initial_yaw_comp])
		elif self.thruster == 3:
			self.thrust3_pub.publish(output)
			self.thruster3_file.writerow([output,self.x_raw-self.initial_x_no_comp, self.y_raw-self.initial_y_no_comp, self.z_raw-self.initial_z_no_comp])#, self.x_comp-self.initial_x_comp,self.y_comp-self.initial_y_comp,self.z_comp-self.initial_z_comp])
			self.thruster9_file.writerow([output,self.roll_raw-self.initial_roll_raw, self.pitch_raw-self.initial_pitch_raw,self.yaw_raw-self.initial_yaw_raw])#,self.roll_comp-self.initial_roll_comp,self.pitch_comp-self.initial_pitch_comp,self.yaw_comp-self.initial_yaw_comp])
		elif self.thruster == 4:
			self.thrust4_pub.publish(output)
			self.thruster4_file.writerow([output,self.x_raw-self.initial_x_no_comp, self.y_raw-self.initial_y_no_comp, self.z_raw-self.initial_z_no_comp])#, self.x_comp-self.initial_x_comp,self.y_comp-self.initial_y_comp,self.z_comp-self.initial_z_comp])
			self.thruster10_file.writerow([output,self.roll_raw-self.initial_roll_raw, self.pitch_raw-self.initial_pitch_raw,self.yaw_raw-self.initial_yaw_raw])#,self.roll_comp-self.initial_roll_comp,self.pitch_comp-self.initial_pitch_comp,self.yaw_comp-self.initial_yaw_comp])
		elif self.thruster == 5:
			self.thrust5_pub.publish(output)
			self.thruster5_file.writerow([output,self.x_raw-self.initial_x_no_comp, self.y_raw-self.initial_y_no_comp, self.z_raw-self.initial_z_no_comp])#, self.x_comp-self.initial_x_comp,self.y_comp-self.initial_y_comp,self.z_comp-self.initial_z_comp])
			self.thruster11_file.writerow([output,self.roll_raw-self.initial_roll_raw, self.pitch_raw-self.initial_pitch_raw,self.yaw_raw-self.initial_yaw_raw])#,self.roll_comp-self.initial_roll_comp,self.pitch_comp-self.initial_pitch_comp,self.yaw_comp-self.initial_yaw_comp])
		elif self.thruster == 6:
			self.thrust6_pub.publish(output)
			self.thruster6_file.writerow([output,self.x_raw-self.initial_x_no_comp, self.y_raw-self.initial_y_no_comp, self.z_raw-self.initial_z_no_comp, self.x_comp-self.initial_x_comp,self.y_comp-self.initial_y_comp,self.z_comp-self.initial_z_comp])
			self.thruster12_file.writerow([output,self.roll_raw-self.initial_roll_raw, self.pitch_raw-self.initial_pitch_raw,self.yaw_raw-self.initial_yaw_raw,self.roll_comp-self.initial_roll_comp,self.pitch_comp-self.initial_pitch_comp,self.yaw_comp-self.initial_yaw_comp])
		else:
			self.thrust1_pub.publish(0.0)
			self.thrust2_pub.publish(0.0)
			self.thrust3_pub.publish(0.0)
			self.thrust4_pub.publish(0.0)
			self.thrust5_pub.publish(0.0)
			self.thrust6_pub.publish(0.0)						

		if self.counter == self.resolution + 1:
			self.counter = 0
			self.thruster += 1

			self.thrust1_pub.publish(0.0)
			self.thrust2_pub.publish(0.0)
			self.thrust3_pub.publish(0.0)
			self.thrust4_pub.publish(0.0)
			self.thrust5_pub.publish(0.0)
			self.thrust6_pub.publish(0.0)

			time.sleep(3)

			self.arm_thrusters()


	def stop_thrusters():
		thrust1_pub = rospy.Publisher('/thruster1_force', Float32, queue_size=1)
		thrust2_pub = rospy.Publisher('/thruster2_force', Float32, queue_size=1)
		thrust3_pub = rospy.Publisher('/thruster3_force', Float32, queue_size=1)
		thrust4_pub = rospy.Publisher('/thruster4_force', Float32, queue_size=1)
		thrust5_pub = rospy.Publisher('/thruster5_force', Float32, queue_size=1)
		thrust6_pub = rospy.Publisher('/thruster6_force', Float32, queue_size=1)

		thrust1_pub.publish(0.0)
		thrust2_pub.publish(0.0)
		thrust3_pub.publish(0.0)
		thrust4_pub.publish(0.0)
		thrust5_pub.publish(0.0)
		thrust6_pub.publish(0.0)

	def save_raw_mag_values(self,data): 
		self.x_raw = data.magnetic_field.x
		self.y_raw = data.magnetic_field.y
		self.z_raw = data.magnetic_field.z

	def save_comp_mag_values(self,data):  #comp_rpy_values stored here
		self.x_comp = data.magnetic_field.x
		self.y_comp = data.magnetic_field.y
		self.z_comp = data.magnetic_field.z

	def save_raw_rpy_values(self, data):	
		self.roll_raw = data.roll
		self.pitch_raw = data.pitch
		self.yaw_raw = data.yaw

	def save_comp_rpy_values(self, data):	
		self.roll_comp = data.comp_roll
		self.pitch_comp = data.comp_pitch
		self.yaw_comp = data.comp_yaw


	def create_files(self):
		date_time = strftime("%d_%m_%y_%H_%M_%S", localtime())
		file_name1 = "/home/andy/catkin_ws/src/anglerfish/t100_thruster/data/thruster1_%s.csv" % date_time
		file_name2 = "/home/andy/catkin_ws/src/anglerfish/t100_thruster/data/thruster2_%s.csv" % date_time
		file_name3 = "/home/andy/catkin_ws/src/anglerfish/t100_thruster/data/thruster3_%s.csv" % date_time
		file_name4 = "/home/andy/catkin_ws/src/anglerfish/t100_thruster/data/thruster4_%s.csv" % date_time
		file_name5 = "/home/andy/catkin_ws/src/anglerfish/t100_thruster/data/thruster5_%s.csv" % date_time
		file_name6 = "/home/andy/catkin_ws/src/anglerfish/t100_thruster/data/thruster6_%s.csv" % date_time

		file_name7 = "/home/andy/catkin_ws/src/anglerfish/t100_thruster/data/thruster1_RPY_%s.csv" % date_time
		file_name8 = "/home/andy/catkin_ws/src/anglerfish/t100_thruster/data/thruster2_RPY_%s.csv" % date_time
		file_name9 = "/home/andy/catkin_ws/src/anglerfish/t100_thruster/data/thruster3_RPY_%s.csv" % date_time
		file_name10 = "/home/andy/catkin_ws/src/anglerfish/t100_thruster/data/thruster4_RPY_%s.csv" % date_time
		file_name11 = "/home/andy/catkin_ws/src/anglerfish/t100_thruster/data/thruster5_RPY_%s.csv" % date_time
		file_name12 = "/home/andy/catkin_ws/src/anglerfish/t100_thruster/data/thruster6_RPY_%s.csv" % date_time
		
		self.thruster1_file = csv.writer(open(file_name1,'w'))
		self.thruster1_file.writerow(["duty cycle","x_mag_raw","y_mag_raw","z_mag_raw", "x_mag_corr","y_mag_corr","z_mag_corr"])
		self.thruster2_file = csv.writer(open(file_name2,'w'))
		self.thruster2_file.writerow(["duty cycle","x_mag_raw","y_mag_raw","z_mag_raw"])#, "x_mag_corr","y_mag_corr","z_mag_corr"])		
		self.thruster3_file = csv.writer(open(file_name3,'w'))
		self.thruster3_file.writerow(["duty cycle","x_mag_raw","y_mag_raw","z_mag_raw"])#, "x_mag_corr","y_mag_corr","z_mag_corr"])
		self.thruster4_file = csv.writer(open(file_name4,'w'))
		self.thruster4_file.writerow(["duty cycle","x_mag_raw","y_mag_raw","z_mag_raw"])#, "x_mag_corr","y_mag_corr","z_mag_corr"])
		self.thruster5_file = csv.writer(open(file_name5,'w'))
		self.thruster5_file.writerow(["duty cycle","x_mag_raw","y_mag_raw","z_mag_raw"])#, "x_mag_corr","y_mag_corr","z_mag_corr"])
		self.thruster6_file = csv.writer(open(file_name6,'w'))
		self.thruster6_file.writerow(["duty cycle","x_mag_raw","y_mag_raw","z_mag_raw"])#, "x_mag_corr","y_mag_corr","z_mag_corr"])

		self.thruster7_file = csv.writer(open(file_name7,'w'))
		self.thruster7_file.writerow(["duty cycle","roll_raw","pitch_raw","yaw_raw"])#, "roll_corr","pitch_corr","yaw_corr"])
		self.thruster8_file = csv.writer(open(file_name8,'w'))
		self.thruster8_file.writerow(["duty cycle","roll_raw","pitch_raw","yaw_raw"])#, "roll_corr","pitch_corr","yaw_corr"])
		self.thruster9_file = csv.writer(open(file_name9,'w'))
		self.thruster9_file.writerow(["duty cycle","roll_raw","pitch_raw","yaw_raw"])#, "roll_corr","pitch_corr","yaw_corr"])
		self.thruster10_file = csv.writer(open(file_name10,'w'))
		self.thruster10_file.writerow(["duty cycle","roll_raw","pitch_raw","yaw_raw"])#, "roll_corr","pitch_corr","yaw_corr"])
		self.thruster11_file = csv.writer(open(file_name11,'w'))
		self.thruster11_file.writerow(["duty cycle","roll_raw","pitch_raw","yaw_raw"])#, "roll_corr","pitch_corr","yaw_corr"])
		self.thruster12_file = csv.writer(open(file_name12,'w'))
		self.thruster12_file.writerow(["duty cycle","roll_raw","pitch_raw","yaw_raw"])#, "roll_corr","pitch_corr","yaw_corr"])

	def __init__(self):
		self.rate = rospy.Rate(25)

		self.thrust1_pub = rospy.Publisher('/thruster1_force', Float32, queue_size=1)
		self.thrust2_pub = rospy.Publisher('/thruster2_force', Float32, queue_size=1)
		self.thrust3_pub = rospy.Publisher('/thruster3_force', Float32, queue_size=1)
		self.thrust4_pub = rospy.Publisher('/thruster4_force', Float32, queue_size=1)
		self.thrust5_pub = rospy.Publisher('/thruster5_force', Float32, queue_size=1)
		self.thrust6_pub = rospy.Publisher('/thruster6_force', Float32, queue_size=1)

		rospy.Subscriber("/imu/mag_raw", MagneticField, self.save_raw_mag_values) #raw mag values
		rospy.Subscriber('/imu/mag', MagneticField, self.save_comp_mag_values) #compensated mag values
		
		rospy.Subscriber('/mag_raw_rpy', mag_raw, self.save_raw_rpy_values)  #raw rpy values (roll, pitch, yaw)
		rospy.Subscriber("/comp_mag_values", mag_values, self.save_comp_rpy_values) #compensated rpy values (comp_roll, comp_yaw, comp_pitch)

		self.x_raw = 0.0
		self.y_raw = 0.0
		self.z_raw = 0.0

		self.x_comp = 0.0
		self.y_comp = 0.0
		self.z_comp = 0.0

		self.roll_raw = 0.0
		self.pitch_raw = 0.0
		self.yaw_raw = 0.0

		self.roll_comp = 0.0
		self.pitch_comp = 0.0
		self.yaw_comp = 0.0

		self.initial_roll_raw  = 0
		self.initial_pitch_raw = 0
		self.initial_yaw_raw   = 0
		self.initial_roll_comp = 0
		self.initial_pitch_comp= 0
		self.initial_yaw_comp  = 0

		self.window_size = 200
		self.sliderx = [0] * self.window_size
		self.slidery = [0] * self.window_size
		self.sliderz = [0] * self.window_size

		self.no_comp_sliderx = [0] * self.window_size
		self.no_comp_slidery = [0] * self.window_size
		self.no_comp_sliderz = [0] * self.window_size

		self.initial_x_no_comp = 0
		self.initial_y_no_comp = 0
		self.initial_z_no_comp = 0
		self.initial_x_comp    = 0
		self.initial_y_comp    = 0
		self.initial_z_comp    = 0

		self.counter = 0
		self.thruster = 1

		self.resolution = 200.0
		self.forward_thrust_force = 1 #2.36 kg forward
		self.reverse_thrust_force = -1 #1.82 kg reverse
		
		self.direction = 'forward'
		self.init_rpy = 1
		self.init_rpy2 = 1
		self.roll_diff = 0.0
		self.pitch_diff = 0.0
		self.yaw_diff = 0.0

		self.new_roll = 0.0
		self.new_pitch = 0.0
		self.new_yaw = 0.0
		self.old_roll = 0.0
		self.old_pitch = 0.0
		self.old_yaw = 0.0
		self.initial_roll = 0.0
		self.initial_pitch = 0.0
		self.initial_yaw = 0.0
		self.initial_roll_raw = 0.0
		self.initial_pitch_raw = 0.0
		self.initial_yaw_raw = 0.0
		self.stable = False
		self.stable_rate = rospy.Rate(20)
		self.sensitivity = 0.05
		self.init_rpy = 0
		self.init_rpy2 = 0

		self.mag_rate = rospy.Rate(25) #15

		rospy.loginfo("Waiting for initial values")
		while(self.x_raw == 0.0 and self.x_comp == 0.0 and self.roll_raw == 0.0 and self.roll_comp == 0.0):
			self.rate.sleep()
		rospy.loginfo("Initial values acquired")

		self.create_files()
		self.arm_thrusters()		

		while not rospy.is_shutdown():
			if self.counter == 0:
				rospy.loginfo("Starting thruster: %d, direction: %s" % (self.thruster, self.direction))

			if self.direction == 'forward':
				self.forward()
			elif self.direction == 'reverse':
				self.reverse()

			if self.thruster == 7 and self.direction == 'forward':
				self.thruster = 1
				self.direction = 'reverse'
			elif self.thruster == 7 and self.direction == 'reverse':
				self.direction = 'stop'
				rospy.loginfo("Thruster testing complete")
				rospy.signal_shutdown('Complete')

			self.rate.sleep()
	
	rospy.on_shutdown(stop_thrusters)

def main(args):
	rospy.init_node('thruster_test', anonymous=False)

	start_test()

        try:
		rospy.spin()
        except rospy.ROSInterruptException:
		print "Shutting down"
                pass
	


if __name__ == '__main__':
	main(sys.argv)
