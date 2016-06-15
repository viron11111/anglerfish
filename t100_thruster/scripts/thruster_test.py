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
from anglerfish.msg import t100_thruster_feedback
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import MagneticField


class start_test():

	def arm_thrusters(self):
		rospy.loginfo("Arming thrusters...")
		self.thrust1_pub.publish(0.0)
		self.thrust2_pub.publish(0.0)
		self.thrust3_pub.publish(0.0)
		self.thrust4_pub.publish(0.0)
		self.thrust5_pub.publish(0.0)
		self.thrust6_pub.publish(0.0)
		time.sleep(1)

	def forward(self):
		output = (float(self.counter)/self.resolution) * self.forward_thrust_force

		self.counter += 1

		if self.thruster == 1:
			self.thrust1_pub.publish(output)
			self.thruster1_file.writerow([self.x_compensated-self.x_average,self.y_compensated-self.y_average,self.z_compensated-self.z_average,output])
		elif self.thruster == 2:
			self.thrust2_pub.publish(output)
			self.thruster2_file.writerow([self.x_compensated-self.x_average,self.y_compensated-self.y_average,self.z_compensated-self.z_average,output])
		elif self.thruster == 3:
			self.thrust3_pub.publish(output)
			self.thruster3_file.writerow([self.x_compensated-self.x_average,self.y_compensated-self.y_average,self.z_compensated-self.z_average,output])
		elif self.thruster == 4:
			self.thrust4_pub.publish(output)
			self.thruster4_file.writerow([self.x_compensated-self.x_average,self.y_compensated-self.y_average,self.z_compensated-self.z_average,output])
		elif self.thruster == 5:
			self.thrust5_pub.publish(output)
			self.thruster5_file.writerow([self.x_compensated-self.x_average,self.y_compensated-self.y_average,self.z_compensated-self.z_average,output])
		elif self.thruster == 6:
			self.thrust6_pub.publish(output)
			self.thruster6_file.writerow([self.x_compensated-self.x_average,self.y_compensated-self.y_average,self.z_compensated-self.z_average,output])
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
			time.sleep(1)

	def reverse(self):
		output = (float(self.counter)/self.resolution) * self.reverse_thrust_force

		self.counter += 1

		if self.thruster == 1:
			self.thrust1_pub.publish(output)
			self.thruster1_file.writerow([self.x_compensated-self.x_average,self.y_compensated-self.y_average,self.z_compensated-self.z_average,output])
		elif self.thruster == 2:
			self.thrust2_pub.publish(output)
			self.thruster2_file.writerow([self.x_compensated-self.x_average,self.y_compensated-self.y_average,self.z_compensated-self.z_average,output])
		elif self.thruster == 3:
			self.thrust3_pub.publish(output)
			self.thruster3_file.writerow([self.x_compensated-self.x_average,self.y_compensated-self.y_average,self.z_compensated-self.z_average,output])
		elif self.thruster == 4:
			self.thrust4_pub.publish(output)
			self.thruster4_file.writerow([self.x_compensated-self.x_average,self.y_compensated-self.y_average,self.z_compensated-self.z_average,output])
		elif self.thruster == 5:
			self.thrust5_pub.publish(output)
			self.thruster5_file.writerow([self.x_compensated-self.x_average,self.y_compensated-self.y_average,self.z_compensated-self.z_average,output])
		elif self.thruster == 6:
			self.thrust6_pub.publish(output)
			self.thruster6_file.writerow([self.x_compensated-self.x_average,self.y_compensated-self.y_average,self.z_compensated-self.z_average,output])
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
			time.sleep(1)			

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

	def save_mag_values(self,data):
		self.x_compensated = data.magnetic_field.x
		self.y_compensated = data.magnetic_field.y
		self.z_compensated = data.magnetic_field.z		

		for i in range(self.window_size-1, -1, -1):
			self.sliderx[i] = self.sliderx[i-1]
			self.slidery[i] = self.slidery[i-1]
			self.sliderz[i] = self.sliderz[i-1]
		self.sliderx[0] = self.x_compensated
		self.slidery[0] = self.y_compensated
		self.sliderz[0] = self.z_compensated

		self.mag_avg_x = sum(self.sliderx)/self.window_size
		self.mag_avg_y = sum(self.slidery)/self.window_size
		self.mag_avg_z = sum(self.sliderz)/self.window_size

	def create_files(self):
		date_time = strftime("%d_%m_%y_%H_%M_%S", localtime())
		file_name1 = "/home/andy/catkin_ws/src/anglerfish/t100_thruster/data/thruster1.csv"#,date_time)
		file_name2 = "/home/andy/catkin_ws/src/anglerfish/t100_thruster/data/thruster2.csv"#,date_time)
		file_name3 = "/home/andy/catkin_ws/src/anglerfish/t100_thruster/data/thruster3.csv"#,date_time)
		file_name4 = "/home/andy/catkin_ws/src/anglerfish/t100_thruster/data/thruster4.csv"#,date_time)
		file_name5 = "/home/andy/catkin_ws/src/anglerfish/t100_thruster/data/thruster5.csv"#,date_time)
		file_name6 = "/home/andy/catkin_ws/src/anglerfish/t100_thruster/data/thruster6.csv"#,date_time)
		
		self.thruster1_file = csv.writer(open(file_name1,'w'))
		self.thruster1_file.writerow(["x_mag","y_mag","z_mag","force (kg)"])
		self.thruster2_file = csv.writer(open(file_name2,'w'))
		self.thruster2_file.writerow(["x_mag","y_mag","z_mag","force (kg)"])		
		self.thruster3_file = csv.writer(open(file_name3,'w'))
		self.thruster3_file.writerow(["x_mag","y_mag","z_mag","force (kg)"])
		self.thruster4_file = csv.writer(open(file_name4,'w'))
		self.thruster4_file.writerow(["x_mag","y_mag","z_mag","force (kg)"])
		self.thruster5_file = csv.writer(open(file_name5,'w'))
		self.thruster5_file.writerow(["x_mag","y_mag","z_mag","force (kg)"])
		self.thruster6_file = csv.writer(open(file_name6,'w'))
		self.thruster6_file.writerow(["x_mag","y_mag","z_mag","force (kg)"])

	def __init__(self):
		self.thrust1_pub = rospy.Publisher('/thruster1_force', Float32, queue_size=1)
		self.thrust2_pub = rospy.Publisher('/thruster2_force', Float32, queue_size=1)
		self.thrust3_pub = rospy.Publisher('/thruster3_force', Float32, queue_size=1)
		self.thrust4_pub = rospy.Publisher('/thruster4_force', Float32, queue_size=1)
		self.thrust5_pub = rospy.Publisher('/thruster5_force', Float32, queue_size=1)
		self.thrust6_pub = rospy.Publisher('/thruster6_force', Float32, queue_size=1)

		rospy.Subscriber('/imu/mag', MagneticField, self.save_mag_values)

		self.counter = 0
		self.thruster = 1
		self.resolution = 100.0
		self.forward_thrust_force = .25 #2.36 kg forward
		self.reverse_thrust_force = -.25 #1.82 kg reverse
		self.direction = 'forward'

		self.window_size = 100
		self.sliderx = [0] * self.window_size
		self.slidery = [0] * self.window_size
		self.sliderz = [0] * self.window_size

		self.mag_rate = rospy.Rate(15)

		rospy.loginfo("******************** Thruster vs Mag calibration ********************")
		rospy.loginfo("Starting mag averaging, hold sub very still...")
		for i in range (0,self.window_size +1):
			self.mag_rate.sleep()

		self.x_average = self.mag_avg_x
		self.y_average = self.mag_avg_y
		self.z_average = self.mag_avg_z

		rospy.loginfo("Mag averaging complete")

		self.create_files()
		self.arm_thrusters()		

		self.rate = rospy.Rate(100)

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
			#rospy.spin()
	
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
