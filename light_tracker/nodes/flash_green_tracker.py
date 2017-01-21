#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math
from numpy.linalg import inv
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from std_msgs.msg import Bool
from time import localtime,strftime
import datetime
import time

from scipy.misc import imread
from scipy.linalg import norm
from scipy import sum, average

class ThrusterDriver:

	def import_vid(self,data):

		#greenLower = ((64.0/360)*255, 0.15*255, 0.1*255)
		#greenUpper = ((150.0/360)*255, 255, 255)

		greenLower = (40, 100, 0)
		greenUpper = (80, 255, 255)		
		kernel = np.ones((5,5),np.uint8)

		self.bridge = CvBridge()

		image = self.bridge.imgmsg_to_cv2(data, "bgr8")

		img = cv2.blur(image,(5,5))
		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		gray /= 10
		gray *= gray

		gray = cv2.morphologyEx(gray, cv2.MORPH_CLOSE, kernel)
		gray = cv2.morphologyEx(gray, cv2.MORPH_OPEN, kernel)

		ret,thresh = cv2.threshold(gray,100,255,0)

		im2, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

		#cv2.drawContours(image, contours, -1, (0,255,0), 3)

		for cnt in contours:
			if cnt != None:
				M = cv2.moments(cnt)
				cX = int(M["m10"] / M["m00"])
				cY = int(M["m01"] / M["m00"])
				cv2.circle(image, (cX, cY), 7, (255, 0, 0), -1)
				cv2.drawContours(image, contours, -1, (0,0,255), 3)
				#cv2.drawContours(image, [cnt], 0, (0,255,0), 3)
		#self.image1 = img

		#equ = cv2.equalizeHist(img)

		#img = cv2.blur(img,(5,5))

		#hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
		#mask_green = cv2.inRange(hsv, greenLower, greenUpper)

		


		#self.image1 = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

		'''if self.first_frame == 0:
			self.image2 = self.image1
			self.sum = np.zeros([480, 752])
			self.first_frame = 1'''

		#diff1 = np.float32(self.image1) - np.float32(self.image2)

		#self.sum[:] = 

		#self.sum[:] = [x - 50 for x in self.sum[:]] #decay

		#print self.sum[:]
		#time.sleep(1)

		#self.sum[:] += abs(diff1)

		#diff1_array = np.array(diff1, dtype = np.uint8)

		#img1 = self.image1 - self.image2
		#img2 = self.image2 - self.image3

		#kernel = np.ones((5,5),np.uint8)
		#erosion = cv2.erode(img1,kernel,iterations = 2)
		#opening1 = cv2.morphologyEx(erosion, cv2.MORPH_OPEN, kernel)

		#combined = img1 - img2
		#img = cv2.convertScaleAbs(img)

		self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
		#self.image2 = self.image1

	def __init__(self):

		self.first_frame = 0
		self.image2 = 0

		self.image_pub = rospy.Publisher("down_camera_sub",Image, queue_size = 1)
		self.image_sub = rospy.Subscriber("/down/down/image_color",Image,self.import_vid)

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

		rate = rospy.Rate(30)		

		while not rospy.is_shutdown():

			rate.sleep()



def main():
    rospy.init_node('flash_green_tracker', anonymous=False)

    ThrusterDriver()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print "Shutting down"
        pass
    


if __name__ == '__main__':
    main()
