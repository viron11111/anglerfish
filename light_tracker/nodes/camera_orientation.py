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

		self.DEVICE_ID = rospy.get_param('~device_id','/dev/serial/by-id/usb-SparkFun_SFE_9DOF-D21-if00')
		self.imu_pub = rospy.Publisher('imu/camera_tilt', Imu, queue_size = 1)
		self.mag_pub = rospy.Publisher('imu/camera_mag', MagneticField, queue_size = 1)

		imu = Imu()
		mag = MagneticField()

		ser = serial.Serial(self.DEVICE_ID)

		gyro = [0.0,0.0,0.0,0.0]
		magn = [0.0,0.0,0.0,0.0]
		quat = [0.0,1.0,0.0,0.0,0.0]

		rate = rospy.Rate(130)

		while not rospy.is_shutdown():
			line = ser.readline()
			accel = line.split("," , 4)
			if accel[0] == 'A':
				line = ser.readline()
				gyro = line.split("," , 4)
				line = ser.readline()
				magn = line.split("," , 4)
				line = ser.readline()
				quat = line.split("," , 5)


			imu.header.stamp = rospy.Time.now()
			imu.header.frame_id = '/base_link' # i.e. '/odom'
			#pres.child_frame_id = self.child_frame_id # i.e. '/base_footprint'
			imu.angular_velocity.x = float(gyro[1])
			imu.angular_velocity.y = float(gyro[2])
			imu.angular_velocity.z = float(gyro[3])
			imu.angular_velocity_covariance=[.001, 0.0, 0.0,
                                         	 0.0, .001, 0.0,
                                             0.0, 0.0, .001]

			imu.linear_acceleration.x = float(accel[1])
			imu.linear_acceleration.y = float(accel[2])
			imu.linear_acceleration.z = float(accel[3])
			imu.linear_acceleration_covariance = [.01, 0.0, 0.0,
                                         		  0.0, .01, 0.0,
                                                  0.0, 0.0, .01]
			'''imu.orientation.w = float(quat[1])
			imu.orientation.x = float(quat[2])
			imu.orientation.y = float(quat[3])
			imu.orientation.z = float(quat[4])
			imu.orientation_covariance = [.01, 0.0, 0.0,
                                          0.0, .01, 0.0,
                                          0.0, 0.0, .01]    '''

			mag.header.stamp = rospy.Time.now()
			mag.header.frame_id = '/base_link' # i.e. '/odom'
			#pres.child_frame_id = self.child_frame_id # i.e. '/base_footprint'
			mag.magnetic_field.x = float(magn[1])
			mag.magnetic_field.y = float(magn[2])
			mag.magnetic_field.z = float(magn[3])
			mag.magnetic_field_covariance=[.001, 0.0, 0.0,
                                           0.0, .001, 0.0,
                                           0.0, 0.0, .001]                                                                                        

			self.imu_pub.publish(imu)
			self.mag_pub.publish(mag)

			rate.sleep()


def main():
    rospy.init_node('down_camera_imu', anonymous=False)

    track_camera()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print "Shutting down"
        pass
    


if __name__ == '__main__':
    main()
