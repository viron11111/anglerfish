#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math
from numpy.linalg import inv
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu, MagneticField
import tf, tf2_ros
from std_msgs.msg import Float64

import csv
from time import localtime,strftime
import datetime

class ThrusterDriver:

	def create_files(self):
		self.start_time = 0
		self.time_diff = 0

		date_time = strftime("%d_%m_%y_%H_%M_%S", localtime())
		file_name1 = "/home/andy/catkin_ws/src/anglerfish/light_tracker/data/offset_%s.csv" % date_time
		
		self.velocity_file = csv.writer(open(file_name1,'w'))
		self.velocity_file.writerow(["ROV_heading", "Camera_heading", "Difference"])

	def camera_roll(self,data):
		self.roller = data.data*(math.pi/180)

	def camera_pitch(self,data):
		self.pitcher = data.data*(math.pi/180)	
	
	def camera_yaw(self,data):
		self.yaw = data.data*(math.pi/180)			

	def sub_orientation(self, data):

		quat = (
		data.pose.pose.orientation.x,
		data.pose.pose.orientation.y,
		data.pose.pose.orientation.z,
		data.pose.pose.orientation.w)

		euler = tf.transformations.euler_from_quaternion(quat)

		#self.rov_heading = euler[2]
		error_eq = (0.006*math.pow(euler[2],5) - 0.019*math.pow(euler[2],4) 
			- 0.038*math.pow(euler[2], 3) + 0.224*math.pow(euler[2],2) 
			- 0.048*euler[2] - 0.292)

		self.rov_heading = euler[2] - error_eq

	def pressure(self, data):
		self.depth = data.pose.pose.position.z
		#ospy.loginfo(self.depth)

	def magnetic_field(self, data):
		self.cx = data.magnetic_field.x
		self.cy = data.magnetic_field.y
		self.cz = data.magnetic_field.z				

	def camera_imu(self, data):
		quat = (
		data.orientation.x,
		data.orientation.y,
		data.orientation.z,
		data.orientation.w)

		euler = tf.transformations.euler_from_quaternion(quat)

		self.roll = euler[0]
		self.pitch = euler[1]
		#self.yaw = euler[2]'''

		xh=self.cx*math.cos(self.pitch)+self.cy*math.sin(self.pitch)*math.sin(self.roll)+self.cz*math.cos(self.roll)*math.sin(self.pitch)
		yh=self.cy*math.cos(self.roll)-self.cz*math.sin(self.roll)

		#var_compass=math.atan2(-yh,xh)
		self.camera_heading = math.atan2(-yh,xh)

		'''self.holder1 = var_compass# + 1.0
		self.camera_heading = self.holder1*.2 + self.holder2*.2 + self.holder3*.2 + self.holder4*.2 + self.holder5*.2
		self.holder5 = self.holder4
		self.holder4 = self.holder3
		self.holder3 = self.holder2
		self.holder2 = self.holder1	'''	


		'''if(self.camera_heading < 0):
			self.camera_heading += 2*math.pi

		if(self.camera_heading > 2*math.pi):
			self.camera_heading -= 2*math.pi'''

		#rospy.logwarn(self.cx)

		#self.camera_heading -= math.pi

		#self.camera_heading = math.atan2(self.cy, self.cx)

		#self.velocity_file.writerow([self.rov_heading, self.camera_heading, self.rov_heading - self.camera_heading])


		#rospy.logwarn("rov: %f camera: %f diff: %f" % (self.rov_heading,self.camera_heading, self.rov_heading - self.camera_heading))

	def import_vid(self,data):

		biggest_area = 0.0
		biggest_contour = []
		font = cv2.FONT_HERSHEY_SIMPLEX

		self.image_pub = rospy.Publisher("down_camera_sub",Image, queue_size = 1)
		#self.green_pub = rospy.Publisher("green",Image, queue_size = 1)
		#self.white_pub = rospy.Publisher("white",Image, queue_size = 1)
		#self.combined_pub = rospy.Publisher("combined",Image, queue_size = 1)
		#self.pre_pub = rospy.Publisher("pre",Image, queue_size = 1)
		self.bridge = CvBridge()

		#greenLower = (self.H_green_low, self.S_green_low, self.V_green_low)
		#greenUpper = (self.H_green_high, self.S_green_high, self.V_green_high)

		img = self.bridge.imgmsg_to_cv2(data, "bgr8")

		heading = self.rov_heading - self.camera_heading

		degrees = -heading*(180/3.14157)

		cv2.putText(img,'%f' % heading,(10,240), font, 1,(255,255,255),2,cv2.LINE_AA)
		cv2.ellipse(img,(256,256),(100,50),0,0,degrees - 90,255,-1)

		self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))

		self.camera_turn = rospy.Publisher('camera_heading', Float64, queue_size = 1)
		self.rov_turn = rospy.Publisher('rov_heading', Float64, queue_size = 1)

		self.camera_turn.publish(self.camera_heading)
		self.rov_turn.publish(self.rov_heading)

	def __init__(self):
		
		#self.create_files()

		self.depth = 0
		self.threeD_point = [0,0,0]

		self.roll = 0
		self.pitch = 0
		self.yaw = 0

		self.cx = 0
		self.cy = 0
		self.cz = 0

		self.holder1 = 0.0
		self.holder2 = 0.0
		self.holder3 = 0.0
		self.holder4 = 0.0
		self.holder5 = 0.0

		self.camera_heading = 0.0
		self.rov_heading = 0.0

		self.image_sub = rospy.Subscriber("/down/down/image_raw",Image,self.import_vid)
		self.depth_sub = rospy.Subscriber("depth", PoseWithCovarianceStamped, self.pressure)
		self.camera_sub = rospy.Subscriber("/imu/camera_tilt", Imu, self.camera_imu)
		self.camera_mag_sub = rospy.Subscriber("/imu/mag_camera", MagneticField, self.magnetic_field)
		self.rov_orientation_sub = rospy.Subscriber("/odometry/filtered", Odometry, self.sub_orientation)
		self.roll_sub = rospy.Subscriber("camera_roll", Float64, self.camera_roll)
		self.pitch_sub = rospy.Subscriber("camera_pitch", Float64, self.camera_pitch)
		self.yaw_sub = rospy.Subscriber("camera_yaw", Float64, self.camera_yaw)
		self.pose_pub = rospy.Publisher('xy_position', PoseWithCovarianceStamped, queue_size = 1)

		H_green = 160 #160, 80
		S_green = 72 #72
		V_green = 100 #100

		slop = .25  #percent of slop

		self.H_green_low = H_green - H_green*slop
		self.S_green_low = (S_green*2.55) - (S_green*2.55)*slop
		self.V_green_low = (V_green*2.55) - (V_green*2.55)*slop

		if self.H_green_low < 0:
			self.H_green_low = 0
		if self.S_green_low < 0:
			self.S_green_low = 0
		if self.V_green_low < 0:
			self.V_green_low = 0

		self.H_green_high = H_green + H_green*slop
		self.S_green_high = (S_green*2.55) + (S_green*2.55)*slop
		self.V_green_high = (V_green*2.55) + (V_green*2.55)*slop

		if self.H_green_high < 255:
			self.H_green_high = 255
		if self.S_green_high < 255:
			self.S_green_high = 255
		if self.V_green_high < 255:
			self.V_green_high = 255			

		pos = PoseWithCovarianceStamped()
		rate = rospy.Rate(30)

		self.frame_id = '/green_led'

		while not rospy.is_shutdown():

			pos.header.stamp = rospy.Time.now()
			pos.header.frame_id = 'odom' # i.e. '/odom'
			#pres.child_frame_id = self.child_frame_id # i.e. '/base_footprint'

			pos.pose.pose.position.x = -self.threeD_point[1]
			pos.pose.pose.position.y = -self.threeD_point[0]
			pos.pose.pose.position.z = self.depth#self.threeD_point[2]  #comment me out when using pressure sensor!!!!!!

			#pres.pose.pose.position.x = 

			pos.pose.pose.orientation.w = 1.0
			pos.pose.pose.orientation.x = 0
			pos.pose.pose.orientation.y = 0
			pos.pose.pose.orientation.z = 0
			pos.pose.covariance=(np.eye(6)*.05).flatten()

			self.pose_pub.publish(pos)

			rate.sleep()





def main():
    rospy.init_node('green_tracker', anonymous=False)

    ThrusterDriver()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print "Shutting down"
        pass
    


if __name__ == '__main__':
    main()


'''		vid = self.bridge.imgmsg_to_cv2(data, "bgr8")

		blank_image_green = np.zeros((480,752,1), np.uint8)
		blank_image_white = np.zeros((480,752,1), np.uint8)
		#blank_image_combined = np.zeros((480,752,1), np.uint8)

		#vid = cv2.imread(data)
		#vid = cv2.GaussianBlur(vid,(1,1),1)
		hsv = cv2.cvtColor(vid, cv2.COLOR_BGR2HSV)

		mask_green = cv2.inRange(hsv, greenLower, greenUpper)
		mask_green = cv2.erode(mask_green, None, iterations=1) #1
		mask_green = cv2.dilate(mask_green, None, iterations=7) #7
		mask_green = cv2.erode(mask_green, None, iterations=5) #5
		mask_green = 255 - mask_green

		image, contours, hierarchy = cv2.findContours(mask_green,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		
		listed_contours = []

		for cnt in contours:
			M = cv2.moments(cnt)
			if cnt != None and M['m00'] != 0: 	

				listed_contours.append(cnt)
				cv2.drawContours(blank_image_green, listed_contours, -1, (255), -1)			

				x,y,w,h = cv2.boundingRect(cnt)
				aspect_ratio = float(w)/h

				if cv2.contourArea(cnt) > 50 and cv2.contourArea(cnt) < 3000 and aspect_ratio > .5 and aspect_ratio < 1.5: # and cv2.arcLength(cnt,True):

					listed_contours.append(cnt)

					#cx = int(M['m10']/M['m00'])
					#cy = int(M['m01']/M['m00'])

					#cv2.drawContours(blank_image_green, listed_contours, -1, (255), -1)


		mask_white = cv2.inRange(hsv, whiteLower, whiteUpper)
		mask_white = cv2.erode(mask_white, None, iterations=3) #3
		mask_white = cv2.dilate(mask_white, None, iterations=3) #3
		#mask_white = cv2.erode(mask_white, None, iterations=5) #3
		mask_white = 255 - mask_white

		image, contours, hierarchy = cv2.findContours(mask_white,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		
		listed_contours = []

		for cnt in contours:
			M = cv2.moments(cnt)
			if cnt != None and M['m00'] != 0: 				

				x,y,w,h = cv2.boundingRect(cnt)
				aspect_ratio = float(w)/h

				if cv2.contourArea(cnt) > 100 and cv2.contourArea(cnt) < 3000 and aspect_ratio > .4 and aspect_ratio < 1.6: # and cv2.arcLength(cnt,True):

					listed_contours.append(cnt)

					cx = int(M['m10']/M['m00'])
					cy = int(M['m01']/M['m00'])
					
					img = cv2.circle(img, (cx,cy), 2, (0,0,255), 5)
					img = cv2.line(img, (376,240), (cx,cy), (255,0,0), 5)

					cv2.drawContours(blank_image_white, listed_contours, -1, (255), -1)

		blank_image_combined = cv2.bitwise_and(blank_image_white, blank_image_green)
		self.combined_pub.publish(self.bridge.cv2_to_imgmsg(blank_image_combined, "8UC1"))

		image, contours, hierarchy = cv2.findContours(blank_image_combined,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		
		listed_contours = []

		for cnt in contours:

			if cnt != None and M['m00'] != 0: 	
				if cv2.contourArea(cnt)	> biggest_area:
					biggest_area = cv2.contourArea(cnt)
					M = cv2.moments(cnt)
					#rospy.loginfo(biggest_contour)

		if M['m00'] != 0 and cnt != None and biggest_area != 0.0:
			
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])

			#img = cv2.circle(img, (cx,cy), 2, (0,0,255), 5)
			#img = cv2.line(img, (376,240), (cx,cy), (255,0,0), 5)
			#if cv2.arcLength(cnt,True) == True:

			#rospy.loginfo (cv2.arcLength(cnt,True))

			#K = ([[607.830496, 0.000000, 364.584318], [0.000000, 605.641275, 210.198029], [0.000000, 0.000000, 1.000000]])
			K = ([[385.839243, 0.000000, 373.724209], [0.000000, 383.636586, 223.867784], [0.000000, 0.000000, 1.000000]])

			target_point = [[cx],[cy],[1]]
			#ref_point =

			D_vec = np.dot(inv(K), target_point) #hypotenuse vector, no length
			L_vec = [0,0,1]   #vector from center of camera

			mag_D_vec = np.linalg.norm(D_vec) 
			mag_L_vec = np.linalg.norm(L_vec)

			D_vec = D_vec/mag_D_vec

			vec_dot = np.dot(L_vec, D_vec)


			vector_angle = math.acos(vec_dot/(mag_D_vec*mag_L_vec))

			#soh cah toa
			#L = -1.98
			L = self.depth
			actual_distance_from_center = math.tan(vector_angle)*L

			hypotenuse = actual_distance_from_center/math.asin(vector_angle)

			self.threeD_point = D_vec*hypotenuse
			orig_3d = D_vec*hypotenuse

			#*****************************************************************************************************
			#for rotation 90 degrees
			self.threeD_point[0] = orig_3d[0]*math.cos(1.571) - orig_3d[1]*math.sin(1.571)
			self.threeD_point[1] = orig_3d[0]*math.sin(1.571) + orig_3d[1]*math.cos(1.571) 
			self.threeD_point[2] = orig_3d[2]
			#*****************************************************************************************************
			

			angle = math.atan2(cy-240,cx-376)
			img = cv2.ellipse(img,(376,240),(100,100),0, 0, math.degrees(angle), (0,255,0), 5)

			
		#self.green_pub.publish(self.bridge.cv2_to_imgmsg(blank_image_green, "8UC1"))					
		#self.white_pub.publish(self.bridge.cv2_to_imgmsg(blank_image_white, "8UC1"))'''