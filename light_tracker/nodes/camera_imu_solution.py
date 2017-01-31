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
import tf, tf2_ros
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

class track_camera:

	def __init__(self):

		self.DEVICE_ID = rospy.get_param('~device_id','/dev/serial/by-id/usb-SparkFun_SFE_9DOF-D21-if00')
		self.imu_pub = rospy.Publisher('imu/camera_tilt', Imu, queue_size = 1)
		self.mag_pub = rospy.Publisher("/imu/mag_camera", MagneticField, queue_size=1)
		self.roll_pub = rospy.Publisher("camera_roll", Float64, queue_size = 1)
		self.pitch_pub = rospy.Publisher("camera_pitch", Float64, queue_size = 1)
		self.yaw_pub = rospy.Publisher("camera_yaw", Float64, queue_size = 1)
		#self.mag_pub = rospy.Publisher('imu/camera_mag', MagneticField, queue_size = 1)

		head1 = 0
		head2 = 0
		head3 = 0
		head4 = 0
		head5 = 0
		head6 = 0

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
				line = ser.readline()
				rpy = line.split("," , 4)

			magneticsphere = ([1.0974375136949293, 0.008593465160122918, 0.0003032211129407881],
							 [0.008593465160122857, 1.1120651652148803, 0.8271491584066613],
							 [0.00030322111294077105, 0.0926004380061327, 0.5424720769663267])
			
			bias = ([-0.021621641870448665],
			        [-0.10661356710756408],
			        [0.4377993330407842])
			
			raw_values = ([float(magn[1])],
						  [float(magn[2])],
						  [float(magn[3])])

			shift = ([raw_values[0][0] - bias[0][0]],
				[raw_values[1][0] - bias[1][0]],
				[raw_values[2][0] - bias[2][0]])


			magneticfield = np.dot(magneticsphere, shift)
			#rospy.loginfo(magneticfield)

			imu.header.stamp = rospy.Time.now()
			imu.header.frame_id = '/base_link' # i.e. '/odom'
			#pres.child_frame_id = self.child_frame_id # i.e. '/base_footprint'
			imu.angular_velocity.x = float(gyro[1])
			imu.angular_velocity.y = float(gyro[2])
			imu.angular_velocity.z = float(gyro[3])
			imu.angular_velocity_covariance=[.01, 0.0, 0.0,
                                         	 0.0, .01, 0.0,
                                             0.0, 0.0, .01]

			imu.linear_acceleration.x = float(accel[1])
			imu.linear_acceleration.y = float(accel[2])
			imu.linear_acceleration.z = float(accel[3])
			imu.linear_acceleration_covariance = [.01, 0.0, 0.0,
                                         		  0.0, .01, 0.0,
                                                  0.0, 0.0, .01]

			mag.header.stamp = rospy.Time.now()
			mag.header.frame_id = '/base_link' # i.e. '/odom'                                                  

			mag.magnetic_field.x = float(magn[1])/100.0
			mag.magnetic_field.y = float(magn[2])/100.0
			mag.magnetic_field.z = float(magn[3])/100.0
			mag.magnetic_field_covariance = [.01, 0.0, 0.0,
                                         	0.0, .01, 0.0,
                                            0.0, 0.0, .01]

			imu.orientation.w = float(quat[1])
			imu.orientation.x = float(quat[2])
			imu.orientation.y = float(quat[3])
			imu.orientation.z = float(quat[4])
			imu.orientation_covariance = [.01, 0.0, 0.0,
                                          0.0, .01, 0.0,
                                          0.0, 0.0, .01]

			#magn[2] = -float(magn[2])
			#heading = math.atan2(float(magn[2]),float(magn[1]))*(180/math.pi)
			#heading = math.atan2(float(magn[2]),float(magn[1]))
			#if heading < 0:
			#	heading += 360                                                                                                                          
			#rospy.loginfo(head[1])

			self.roll_pub.publish(float(rpy[1]))
			self.pitch_pub.publish(float(rpy[2]))
			self.pitch_pub.publish(float(rpy[3]))
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