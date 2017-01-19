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

		self.bridge = CvBridge()

		img = self.bridge.imgmsg_to_cv2(data, "bgr8")

		self.image1 = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

		if self.first_frame == 0:
			self.image2 = self.image1
			self.sum = np.zeros([480, 752])
			self.first_frame = 1

		diff1 = np.float32(self.image1) - np.float32(self.image2)

		#self.sum[:] = 

		#self.sum[:] = [x - 50 for x in self.sum[:]] #decay

		for i in range(len(self.sum)):
			for j in range(len(self.sum[i])):
				self.sum[i][j] - 100
				if self.sum[i][j] < 0:
					self.sum[i][j] = 0


		#print self.sum[:]
		#time.sleep(1)

		self.sum[:] += abs(diff1)

		diff1_array = np.array(self.sum, dtype = np.uint8)

		#img1 = self.image1 - self.image2
		#img2 = self.image2 - self.image3

		#kernel = np.ones((5,5),np.uint8)
		#erosion = cv2.erode(img1,kernel,iterations = 2)
		#opening1 = cv2.morphologyEx(erosion, cv2.MORPH_OPEN, kernel)

		#combined = img1 - img2
		#img = cv2.convertScaleAbs(img)


		self.image_pub.publish(self.bridge.cv2_to_imgmsg(diff1_array, "8UC1"))
		self.image2 = self.image1

	def __init__(self):

		self.first_frame = 0
		self.image2 = 0

		self.image_pub = rospy.Publisher("down_camera_sub",Image, queue_size = 1)
		self.image_sub = rospy.Subscriber("/down/down/image_color",Image,self.import_vid)

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
