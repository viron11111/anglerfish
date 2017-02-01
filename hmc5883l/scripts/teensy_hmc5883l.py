#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image, Imu, MagneticField
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math
from numpy.linalg import inv
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
import serial

class track_camera:

	def __init__(self):

		self.DEVICE_ID = rospy.get_param('~device_id','/dev/serial/by-id/usb-Teensyduino_USB_Serial_1577200-if00')
		self.mag_pub = rospy.Publisher('imu/mag', MagneticField, queue_size = 1)

		mag = MagneticField()

		ser = serial.Serial(self.DEVICE_ID)

		magn = [0.0,0.0,0.0,0.0]

		rate = rospy.Rate(60)

		while not rospy.is_shutdown():
			ser.write('r')
			line = ser.readline()
			magn = line.split("," , 4)
			#rospy.loginfo(magn)
			if magn[0] == 'XYZ':
    				mag.header.stamp = rospy.Time.now()
				mag.header.frame_id = '/base_link' # i.e. '/odom'
				mag.magnetic_field.x = float(magn[1]) #/100.0
				mag.magnetic_field.y = float(magn[2]) #/100.0
				mag.magnetic_field.z = float(magn[3]) #/100.0
				mag.magnetic_field_covariance=[.1, 0.0, 0.0,
                                	        	      0.0, .1, 0.0,
                                        	  	      0.0, 0.0, .1]                                                                                        
			self.mag_pub.publish(mag)

			rate.sleep()


def main():
    rospy.init_node('teensy_hmc5883l_magnetometer', anonymous=False)

    track_camera()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print "Shutting down"
        pass
    


if __name__ == '__main__':
    main()
