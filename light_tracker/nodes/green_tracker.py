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

		greenLower = (0, 0, 220)
		greenUpper = (0, 0, 255)

		img = self.bridge.imgmsg_to_cv2(data, "bgr8")

		vid = self.bridge.imgmsg_to_cv2(data, "bgr8")

		#vid = cv2.imread(data)
		vid = cv2.GaussianBlur(vid,(9,9),1)
		hsv = cv2.cvtColor(vid, cv2.COLOR_BGR2HSV)

		mask = cv2.inRange(hsv, greenLower, greenUpper)
		mask = cv2.erode(mask, None, iterations=6)
		mask = cv2.dilate(mask, None, iterations=6)

		image, contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		img = cv2.drawContours(img, contours, -1, (0,0,0), 3)

		for cnt in contours:

			if cnt != None: 
				M = cv2.moments(cnt)
				cx = int(M['m10']/M['m00'])
				cy = int(M['m01']/M['m00'])

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
				L = 1.5254



				actual_distance_from_center = math.tan(vector_angle)*L

				hypotenuse = actual_distance_from_center/math.asin(vector_angle)

				threeD_point = D_vec*hypotenuse


				#rospy.loginfo(mag_D_vec)
				#rospy.loginfo("angle %f degrees", math.degrees(vector_angle))
				rospy.loginfo("3d %f %f %f meters", *threeD_point)
				#rospy.loginfo("distance %f", actual_distance_from_center)

				angle = math.atan2(cy-240,cx-376)
				img = cv2.ellipse(img,(376,240),(100,100),0, 0, math.degrees(angle), (0,255,0), 5)
				img = cv2.circle(img, (cx,cy), 2, (0,0,255), 5)
				img = cv2.line(img, (376,240), (cx,cy), (255,0,0), 5)

		self.blur_pub.publish(self.bridge.cv2_to_imgmsg(mask, "8UC1"))
		#self.image_pub.publish(self.bridge.cv2_to_imgmsg(imgray, "8UC1"))
		self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))

	def __init__(self):
		self.depth = 0
		self.image_sub = rospy.Subscriber("/down/down/image_raw",Image,self.import_vid)
		self.depth_sub = rospy.Subscriber("depth", PoseWithCovarianceStamped, self.pressure)



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