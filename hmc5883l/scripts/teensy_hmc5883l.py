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

	def checkEqual(self, lst):
		return lst[1:] == lst[:-1]

	def __init__(self):

		magx = [1,2,3,4]
		magy = [1,2,3,4]
		magz = [1,2,3,4]

		self.DEVICE_ID = rospy.get_param('~device_id','/dev/serial/by-id/usb-Teensyduino_USB_Serial_1558050-if00')
		self.mag_pub = rospy.Publisher('imu/mag', MagneticField, queue_size = 1)

		mag = MagneticField()

		ser = serial.Serial(self.DEVICE_ID)

		magn = [0.0,0.0,0.0,0.0]

		rate = rospy.Rate(70)

		while not rospy.is_shutdown():
			ser.write('r')
			line = ser.readline()
			magn = line.split("," , 4)
			#rospy.loginfo(magn)
			if magn[0] == 'XYZ':
				x_raw = float(magn[1]) /100.0
				y_raw = float(magn[2]) /100.0
				z_raw = float(magn[3]) /100.0
				scale = np.array([[0.862921585854914, -0.2492307368595151, 0.15942070261516456],
			        		[-0.2492307368595149, 1.2281817269897255, 0.10028203353241516], 
			              		[0.15942070261516447, 0.10028203353241523, 1.0502676905493935]])

				corrected = np.dot([x_raw, y_raw, z_raw], scale) + np.array([0.34930748967355096, 
			                                                                           0.15355419614296578,
			                                                                            -0.1804685182253677])


    				mag.header.stamp = rospy.Time.now()
				mag.header.frame_id = '/base_link' # i.e. '/odom'
				mag.magnetic_field.x = corrected[0] 
				mag.magnetic_field.y = corrected[1]
				mag.magnetic_field.z = corrected[2]
				mag.magnetic_field_covariance=[.01, 0.0, 0.0,
                                	        	      0.0, .01, 0.0,
                                        	  	      0.0, 0.0, .01]                                                                                        
			self.mag_pub.publish(mag)

			#heading = math.atan2(mag.magnetic_field.y,mag.magnetic_field.x)
			'''for i in range(3,0,-1):
				magx[i] = magx[i-1]
			magx[0] = mag.magnetic_field.x

                        for i in range(3,0,-1):
                                magy[i] = magy[i-1]
                        magy[0] = mag.magnetic_field.y

                        for i in range(3,0,-1):
                                magz[i] = magz[i-1]
                        magz[0] = mag.magnetic_field.z

			if self.checkEqual(magx) == True:
				rospy.logerr("MAGX value is static: %f" % magx[0])
                        if self.checkEqual(magy) == True:
                                rospy.logerr("MAGY value is static: %f" % magy[0])
                        if self.checkEqual(magz) == True:
                                rospy.logerr("MAGZ value is static: %f" % magz[0])'''

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
