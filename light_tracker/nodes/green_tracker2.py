#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math
from numpy.linalg import inv
import geometry_msgs.msg 
from geometry_msgs.msg import WrenchStamped, Twist, PoseStamped
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu, MagneticField
import tf, tf2_ros
from std_msgs.msg import Float64
from orientation_library import transformations as trns

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

	def sub_orientation(self, data):

		quat = (
		data.pose.pose.orientation.x,
		data.pose.pose.orientation.y,
		data.pose.pose.orientation.z,
		data.pose.pose.orientation.w)

		euler = tf.transformations.euler_from_quaternion(quat)

		self.rov_heading = euler[2]

	def pressure(self, data):
		self.depth = data.pose.pose.position.z

	def camera_imu(self, data):
		quat = (
		data.orientation.x,
		data.orientation.y,
		data.orientation.z,
		data.orientation.w)

		(self.roll, self.pitch, self.yaw) = tf.transformations.euler_from_quaternion(quat)

		#self.roll = euler[0]
		#self.pitch = euler[1]
		#self.yaw = euler[2]

		self.camera_heading = self.yaw

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

	def import_vid(self,data):

		biggest_area = 0.0
		biggest_contour = []
		font = cv2.FONT_HERSHEY_SIMPLEX

		self.image_pub = rospy.Publisher("down_camera_pub",Image, queue_size = 1)
		#self.green_pub = rospy.Publisher("green",Image, queue_size = 1)
		#self.white_pub = rospy.Publisher("white",Image, queue_size = 1)
		#self.combined_pub = rospy.Publisher("combined",Image, queue_size = 1)
		#self.pre_pub = rospy.Publisher("pre",Image, queue_size = 1)

		greenLower = (40, 100, 0)
		greenUpper = (80, 255, 255)		
		kernel = np.ones((5,5),np.uint8)

		self.bridge = CvBridge()

		#greenLower = (self.H_green_low, self.S_green_low, self.V_green_low)
		#greenUpper = (self.H_green_high, self.S_green_high, self.V_green_high)

		image = self.bridge.imgmsg_to_cv2(data, "bgr8")

		rows = 480
		cols = 752
		camera_degrees = -self.camera_heading*(180/3.14157)
		M = cv2.getRotationMatrix2D((cols/2,rows/2),camera_degrees,1)
		image = cv2.warpAffine(image,M,(cols,rows))

		img = cv2.blur(image,(5,5))

		img[:, 0:self.left_column] = 0
		img[:, 752-self.right_column:752] = 0 
		img[0:self.top_row, :] = 0
		img[480-self.bottom_row:480, :] = 0 

		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		gray /= 10
		gray *= gray		

		gray = cv2.morphologyEx(gray, cv2.MORPH_CLOSE, kernel)
		gray = cv2.morphologyEx(gray, cv2.MORPH_OPEN, kernel)	

		'''cimg = image

		circles = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1,20, param1=100,param2=30,minRadius=5,maxRadius=50)

		circles = np.uint16(np.around(circles))

		for i in circles[0,:]:
			# draw the outer circle
			cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)'''

		ret,thresh = cv2.threshold(gray,130,255,0)  #100

		#self.image_pub.publish(self.bridge.cv2_to_imgmsg(thresh, "8UC1")) #"8UC1"))

		im2, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

		area = 0
		old_area= 0
		biggest_cnt = None
		smallest_area = 1
		biggest_area = 1500
		aspect_ratio = 0
		font = cv2.FONT_HERSHEY_SIMPLEX

		for cnt in contours:
			if cnt != None:
				x,y,w,h = cv2.boundingRect(cnt)
				aspect_ratio = float(w)/h
				area = cv2.contourArea(cnt)
				if area >= old_area and area < biggest_area and area > smallest_area and aspect_ratio > 0.65 and aspect_ratio < 1.35:
					biggest_cnt = cnt
					old_area = area

		if biggest_cnt != None and old_area < biggest_area and old_area > smallest_area:
			#rospy.loginfo ("area: %f aspect_ratio: %f" % (old_area, aspect_ratio))
			M = cv2.moments(biggest_cnt)
			cx = int(M["m10"] / M["m00"])
			cy = int(M["m01"] / M["m00"])
			self.left_column = cx - 50
			self.right_column = (752 - cx) - 50
			self.top_row = cy - 50
			self.bottom_row = (480 - cy) - 50

			cv2.circle(img, (cx, cy), 7, (0, 0, 255), -1)
			#cv2.drawContours(image, contours, -1, (0,0,255), 3)

			frame_shape = (480,752)

			# Pixel coords about image center (in meters)
			cx_img = cx - frame_shape[1] / 2.0
			cy_img = cy - frame_shape[0] / 2.0
			cx_img *= 6E-6
			cy_img *= 6E-6

			#K = ([[390.160508, 0.000000, 388.917091], [0.000000, 390.160508, 218.490820], [0.000000, 0.000000, 1.000000]])   #wide angle lens, cap on, fixed-aspect-ratio
			#K = ([[391.091019, 0.000000, 384.715490], [0.000000, 389.648800, 219.939006], [0.000000, 0.000000, 1.000000]])  #calibrated with wide angle lens, cap off
			#L = 2.8194  #distance of camera to target in meters
			L = self.depth*1.41874157622	
			target_point = np.array([cx_img,cy_img,0.00234])
			target_point = (target_point * L) / target_point[2]

			#D_vec = np.dot(inv(K), target_point) #hypotenuse vector, no length
			
			D_vec = target_point
			print "D_vec: {}",format(D_vec)
			L_vec = np.array([0,0,L])  #vector from center of camera

			mag_D = np.linalg.norm(D_vec) 
			mag_L = np.linalg.norm(L_vec)
			print mag_D, mag_L
			print mag_D**2 - mag_L**2

			dist  = np.sqrt(mag_D**2 - mag_L**2)
			print "dist: {}".format(dist)
			c = np.array([cx_img, cy_img, 0])
			c = c / np.linalg.norm(c[:2]) * dist
			c[2] = L
			#c[0] -= 0.08

			self.threeD_point = c




			#*****************************************************************************************************
			#for rotation 90 degrees
			#self.threeD_point[0] = -orig_3d[0] #orig_3d[0]*math.cos(1.571 + self.camera_heading) - orig_3d[1]*math.sin(1.571 + self.camera_heading)#orig_3d[0]*math.cos(1.571) - orig_3d[1]*math.sin(1.571)
			#self.threeD_point[1] = -orig_3d[1] #orig_3d[0]*math.sin(1.571 + self.camera_heading) + orig_3d[1]*math.cos(1.571 + self.camera_heading) 
			#self.threeD_point[2] = orig_3d[2]

			x_diff = -self.threeD_point[0] - self.p_desW[0]
			y_diff = -self.threeD_point[1] - self.p_desW[1]

			#cv2.circle(img, (self.p_desW[0], self.p_desW[1]), 7, (0, 255, 255), -1)
			#rospy.loginfo(self.p_desW[0])

			#rospy.loginfo("x_pos_diff: %f y_pos_diff: %f" % (x_diff, y_diff))
			cv2.putText(img,"x_pos_diff: %f y_pos_diff: %f" % (x_diff, y_diff),(10,20), font, 0.5,(255,255,255),1,cv2.LINE_AA)

			#rospy.logwarn(self.threeD_point[0])
		else:
			self.left_column = 0
			self.right_column = 0
			self.top_row = 0
			self.bottom_row = 0
			#rospy.logwarn ("area: %f aspect_ratio: %f" % (old_area, aspect_ratio))

		heading = self.rov_heading - self.camera_heading
		#rospy.logwarn("cam: %f rov: %f" % (self.camera_heading, self.rov_heading))

		degrees = -heading*(180/3.14157)

		#cv2.putText(image,'%f' % heading,(10,20), font, 0.5,(255,255,255),1,cv2.LINE_AA)
		#cv2.ellipse(image,(125,25),(25,25),0,0,degrees - 90,255,-1)

		#cv2.circle(image, (400,200), 1, )

		self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
		#self.image_pub.publish(self.bridge.cv2_to_imgmsg(gray, "8UC1"))

		self.camera_turn = rospy.Publisher('camera_heading', Float64, queue_size = 1)
		self.rov_turn = rospy.Publisher('rov_heading', Float64, queue_size = 1)

		self.camera_turn.publish(self.camera_heading)
		self.rov_turn.publish(self.rov_heading)

	def __init__(self):
		
		#self.create_files()

		self.xmeasfir2 = 0.0
		self.xmeasfir3 = 0.0
		self.xmeasfir4 = 0.0
		self.xmeasfir5 = 0.0

		self.ymeasfir2 = 0.0
		self.ymeasfir3 = 0.0
		self.ymeasfir4 = 0.0
		self.ymeasfir5 = 0.0

		self.depth = 0
		self.threeD_point = [0,0,0]
		self.p_desW = [0,0,0]

		self.left_column = 0
		self.right_column = 0
		self.top_row = 0
		self.bottom_row = 0

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

		#self.image_sub = rospy.Subscriber("camera/image_color",Image,self.import_vid)
		self.image_sub = rospy.Subscriber("camera/image_rect_color",Image,self.import_vid)
		self.depth_sub = rospy.Subscriber("depth", PoseWithCovarianceStamped, self.pressure)
		self.camera_sub = rospy.Subscriber("/imu/razor", Imu, self.camera_imu)
		self.rov_orientation_sub = rospy.Subscriber("/odometry/filtered", Odometry, self.sub_orientation)
		rospy.Subscriber("RC_position", PoseWithCovarianceStamped, self.rc_pos)
		self.pose_pub = rospy.Publisher('/xy_position', PoseWithCovarianceStamped, queue_size = 1)

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
			pos.pose.pose.position.z = self.depth #-0.9271#self.depth#self.threeD_point[2]  #comment me out when using pressure sensor!!!!!!

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