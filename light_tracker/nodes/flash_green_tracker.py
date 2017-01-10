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

class ThrusterDriver:

	def pressure(self, data):
		self.depth = data.pose.pose.position.z
		#ospy.loginfo(self.depth)


	def import_vid(self,data):

		self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")



		#fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_OPEN, kernel)

	def stop_lights():
		light_pub = rospy.Publisher('Green_led', Bool, queue_size = 10)
		rate = rospy.Rate(30)
		for x in range(0, 10):
			light_pub.publish(0)
			rate.sleep()


	def __init__(self):

		self.light_pub = rospy.Publisher('Green_led', Bool, queue_size = 1)

		# Create a blank 300x300 black image
		self.img = np.zeros((640, 480, 3), np.uint8)
		# Fill image with red color(set each pixel to red)
		#self.img[:] = (0, 0, 255)

		self.light_toggle = 0
		self.counter = 0
		self.two_frames = 3

		self.image_sub = rospy.Subscriber("/down/down/image_raw",Image,self.import_vid)
		self.depth_sub = rospy.Subscriber("depth", PoseWithCovarianceStamped, self.pressure)
		self.pose_pub = rospy.Publisher('xy_position', PoseWithCovarianceStamped, queue_size = 1)

		self.raw_pub = rospy.Publisher("toggle_raw",Image, queue_size = 1)

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

		kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
		fgbg = cv2.bgsegm.createBackgroundSubtractorGMG()	

		#gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)

		rate = rospy.Rate(30)

		self.frame_id = '/green_led'

		self.text = 0
		self.bridge = CvBridge()

		value = 255
		whiteLower = (0, 0, 200)
		whiteUpper = (0, 0, 255)		

		while not rospy.is_shutdown():

			self.light_pub.publish(1)
			hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV) #convert it to hsv
			hsv[:,:,2] += value
			#bgr = cv2.cvtColor(self.img, cv2.COLOR_HSV2BGR) #convert it to hsv

			mask_white = cv2.inRange(hsv, whiteLower, whiteUpper)
			#mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_OPEN, kernel)

			fgmask = fgbg.apply(mask_white)

			#fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_OPEN, kernel)

			self.raw_pub.publish(self.bridge.cv2_to_imgmsg(fgmask, "8UC1"))

			rate.sleep()

	rospy.on_shutdown(stop_lights)





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
