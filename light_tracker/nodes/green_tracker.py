#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math
from numpy.linalg import inv
from geometry_msgs.msg import Point, PoseWithCovarianceStamped

class ThrusterDriver:

	def pressure(self, data):
		self.depth = data.pose.pose.position.z
		#ospy.loginfo(self.depth)


	def import_vid(self,data):
		self.image_pub = rospy.Publisher("down_camera_out",Image, queue_size = 1)
		self.blur_pub = rospy.Publisher("blurred",Image, queue_size = 1)
		self.bridge = CvBridge()

		greenLower = (self.H_green_low, self.S_green_low, self.V_green_low)
		greenUpper = (self.H_green_high, self.S_green_high, self.V_green_high)

		whiteLower = (0, 0, 245)
		whiteUpper = (0, 0, 255)

		img = self.bridge.imgmsg_to_cv2(data, "bgr8")

		vid = self.bridge.imgmsg_to_cv2(data, "bgr8")

		#vid = cv2.imread(data)
		#vid = cv2.GaussianBlur(vid,(1,1),1)
		hsv = cv2.cvtColor(vid, cv2.COLOR_BGR2HSV)

		mask = cv2.inRange(hsv, greenLower, greenUpper)
		mask = cv2.erode(mask, None, iterations=1) #3
		mask = cv2.dilate(mask, None, iterations=7) #3
		mask = cv2.erode(mask, None, iterations=5) #3
		mask = 255 - mask

		self.blur_pub.publish(self.bridge.cv2_to_imgmsg(mask, "8UC1"))



		image, contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		

		for cnt in contours:
			M = cv2.moments(cnt)
			if cnt != None and M['m00'] != 0: 				

				if cv2.contourArea(cnt) < 1000 and cv2.contourArea(cnt) > 100 and cv2.arcLength(cnt,True):

					cx = int(M['m10']/M['m00'])
					cy = int(M['m01']/M['m00'])

					cv2.drawContours(img, cnt, -1, (0,0,0), 3)
					img = cv2.circle(img, (cx,cy), 2, (0,0,255), 5)
					img = cv2.line(img, (376,240), (cx,cy), (255,0,0), 5)
					#if cv2.arcLength(cnt,True) == True:

					#rospy.loginfo (cv2.arcLength(cnt,True))

					K = ([[607.830496, 0.000000, 364.584318], [0.000000, 605.641275, 210.198029], [0.000000, 0.000000, 1.000000]])
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
					L = -1.14
					#L = self.depth
					actual_distance_from_center = math.tan(vector_angle)*L

					hypotenuse = actual_distance_from_center/math.asin(vector_angle)

					#self.threeD_point = D_vec*hypotenuse
					orig_3d = D_vec*hypotenuse

					#for rotation 90 degrees
					self.threeD_point[0] = orig_3d[0]*math.cos(1.571) - orig_3d[1]*math.sin(1.571)
					self.threeD_point[1] = orig_3d[0]*math.sin(1.571) + orig_3d[1]*math.cos(1.571) 
					self.threeD_point[2] = orig_3d[2]

					angle = math.atan2(cy-240,cx-376)
					img = cv2.ellipse(img,(376,240),(100,100),0, 0, math.degrees(angle), (0,255,0), 5)

		#self.image_pub.publish(self.bridge.cv2_to_imgmsg(imgray, "8UC1"))
		self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))

	def __init__(self):
		self.depth = 0
		self.threeD_point = [0,0,0]

		self.image_sub = rospy.Subscriber("/down/down/image_raw",Image,self.import_vid)
		self.depth_sub = rospy.Subscriber("depth", PoseWithCovarianceStamped, self.pressure)
		self.pose_pub = rospy.Publisher('xy_position', PoseWithCovarianceStamped, queue_size = 1)

		H_green = 180
		S_green = 72
		V_green = 100

		slop = .5  #percent of slop

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
			pos.pose.pose.position.z = self.threeD_point[2]  #comment me out when using pressure sensor!!!!!!

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
