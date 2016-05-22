#!/usr/bin/env python

import roslib

import sys
import rospy
import cv2
import numpy as np
import math
#import cv2.cv as cv
import colorsys
import time

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class rotate_image:

  def __init__(self):
    self.image_pub = rospy.Publisher("flipped_image",Image, queue_size = 1)
    self.bridge = CvBridge()
    
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)

  def callback(self,data):
   
    try:
      vid = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
      print e    

    vid = cv2.flip(vid,0)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(vid, "bgr8")) 
    except CvBridgeError, e:
      print e

def main(args):
  rospy.init_node('image_flip')
  ri = rotate_image()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
