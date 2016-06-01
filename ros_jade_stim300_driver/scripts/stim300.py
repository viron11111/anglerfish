#!/usr/bin/python

import serial
import numpy as np
import rospy
from geometry_msgs.msg import Quaternion, Vector3
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from std_msgs.msg import Float32
import binascii
import struct
import encodings
import math
import tf
#from scipy import integrate
#import sub8_ros_tools


class Interface(object):
    _baudrate = 921600 #(1000 bits per second)
    _datagram_lengths = {
        chr(0x93): 40 - 2,  #-2 Rate, acceleration, inclination
    }
    # Rate, acceleration, incliination

    def __init__(self):
        self.serial = serial.Serial('/dev/ttyUSB0', baudrate=self._baudrate)
        self.datagram_identifier = chr(0x93) #Rate, acceleration, and inclination
        self.imu_pub = rospy.Publisher('/imu', Imu, queue_size=1)
        self.roll_pub = rospy.Publisher('roll', Float32, queue_size=1)
        self.pitch_pub = rospy.Publisher('pitch', Float32, queue_size=1)
        self.yaw_pub = rospy.Publisher('yaw', Float32, queue_size=1)
        #self.last_msg = Nonesens
        self.gyroDatax = 0
        self.gyroDatay = 0
        self.gyroDataz = 0
        self.anglex = 0
        self.angley = 0
        self.anglez = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        #self.orientation_quaternion = []

    def sync(self):
        char = None
        k = 0
        self.msg = ""
        check = None
        while(1):
            check = self.serial.read(1)
            check = check.encode("hex")
            if  check == "93":
                break
        while k < 37:#self.datagram_identifier:
            char = self.serial.read(1)
            self.msg = self.msg + char
            k += 1
            #char = self.serial.read(1)
        #print self.msg.encode("hex")
        #print self.msg[0:9].encode("hex"), "", self.msg[10:19].encode("hex"), "", self.msg[20:29].encode("hex")

        return k

    def twos_comp(self):
        """compute the 2's compliment of int value val"""
        if (self.data & (1 << (24 - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
            self.data = self.data - (1 << 24)        # compute negative value
        return self.data                         # return positive value as is

    def read_datagram(self):
        msg = self.serial.read(
            self._datagram_lengths[self.datagram_identifier]
        )
 

        start = 0
        
        #print self.msg.encode("hex")
        #print self.msg[start + 0:start + 3][::-1].encode("hex")

        # gyr

        gyro = np.fromstring(
            b'\x00' + self.msg[start + 0:start + 3][::-1] +
            b'\x00' + self.msg[start + 3:start + 6][::-1] +
            b'\x00' + self.msg[start + 6:start + 9][::-1],
            dtype='<i'
        ).astype(np.float32) / (2 ** 21)/128

        start += 10

        # acc

        linear_acceleration = np.fromstring(
            b'\x00' + self.msg[start + 0:start + 3][::-1]  +
            b'\x00' + self.msg[start + 3:start + 6][::-1]  +
            b'\x00' + self.msg[start + 6:start + 9][::-1],
            dtype='<i'
        ).astype(np.float32) / (2 ** 19)/255

        #print linear_acceleration

        start += 10

        # inc
        inclination = np.fromstring(
            b'\x00' + msg[start + 0:start + 3][::-1] +
            b'\x00' + msg[start + 3:start + 6][::-1] +
            b'\x00' + msg[start + 6:start + 9][::-1],
            dtype='<i'
        ).astype(np.float32) / (2 ** 22)

        #for x in range(4,0,-1):
        #    self.gyroHolder[x] = self.gyroHolder[x-1]
            
        #print self.gyroHolder
        #self.gyroHolder[0] = gyro[0]
        #print self.gyroHolder

        #self.gyroData = sum(self.gyroHolder)/5
        #self.gyroData = self.gyroData + 
        #print self.gyroData

        #gyroData_quaternion = tf.transformations.quaternion_from_euler(math.radians((gyro[0]/500)/128), 
        #    math.radians((gyro[1]/500)/128), math.radians((gyro[2]/500)/128))
        #gyroData_quaternion = tf.transformations.quaternion_from_euler((gyro[0]/500)/128, gyro[1]/500, gyro[2]/500)

        self.gyroDatax = (self.gyroDatax + (gyro[0]/1000)) #self.gyroDatax + gyroData_quaternion[0]
        self.gyroDatay = (self.gyroDatay + (gyro[1]/1000)) #self.gyroDatay + gyroData_quaternion[1]
        self.gyroDataz = (self.gyroDataz + (gyro[2]/1000)) #self.gyroDataz + gyroData_quaternion[2]

        self.roll = math.atan2(linear_acceleration[1],linear_acceleration[2])
        #print self.roll

        self.anglex = .98*(self.anglex + math.radians(self.gyroDatax)) + 0.02 * self.roll

        self.gyroDatay = -self.gyroDatay

        self.pitch = -math.atan2(linear_acceleration[0],linear_acceleration[2])

        self.angley = .98*(self.angley + math.radians(self.gyroDatay)) + 0.02 * self.pitch

        self.anglez = (self.anglez + math.radians(self.gyroDataz))

        orientation_quaternion = tf.transformations.quaternion_from_euler(self.anglex, self.angley, self.anglez)#[self.anglex, self.angley, self.anglez, 1] #gyroData_quaternion[3]]

        #rpy_quaternion = tf.transformations.quaternion_from_euler(math.atan2(linear_acceleration[1],linear_acceleration[2]), 
        #    -math.atan2(linear_acceleration[0],linear_acceleration[2]), math.atan2(linear_acceleration[0],linear_acceleration[1]))

        #self.orientation_quaternion = .98*(self.orientation_quaternion + gyroData_quaternion)+.02(rpy_quaternion)

        #print quaternion
        #type(pose) = geometry_msgs.msg.Pose
        #pose.orientation.x = quaternion[0]
        #pose.orientation.y = quaternion[1]
        #pose.orientation.z = quaternion[2]
        #pose.orientation.w = quaternion[3]
        #print self.angle


        #print math.radians(self.angle)

        #self.gyroData = self.gyroData + gyro[0]
        #print self.gyroData
        #angle = 0.98(angle + gyroData)

        #angle = 0.98*(angle + gyro[0]*dt) + 0.02*linear_acceleration[0]


#header=sub8_ros_tools.make_header(),

        imu_msg = Imu(            
            header= Header(
                frame_id = 'base_link',
                stamp=rospy.get_rostime()
            ),
            angular_velocity=Vector3(*gyro),
            linear_acceleration=Vector3(*linear_acceleration),
            orientation=Quaternion(*orientation_quaternion)
        )

        self.imu_pub.publish(imu_msg)
        self.roll_pub.publish(self.pitch)
        self.pitch_pub.publish(self.angley)
        self.yaw_pub.publish(self.gyroDatay)
        
        br = tf.TransformBroadcaster()
        br.sendTransform((0, 0, 0), orientation_quaternion, rospy.Time.now(), "stim_imu", "world")

    def run(self):
        l = self.sync()
        l = 38
        if l != 38:
            return
        self.read_datagram()


    def holder(self):
        self.gyroDatax = self.gyroDatax+(gyro[0]/500.0) #for 500 Hz rate

        self.gyroDatax = self.gyroDatax/128.0  #128 was discovered through trial and error


        self.roll = math.atan2(linear_acceleration[1],linear_acceleration[2])

        self.anglex = 0.98*(self.anglex + math.radians(self.gyroDatax)) + 0.02 * self.roll


        self.gyroDatay = self.gyroDatay+(gyro[1]/500) #for 500 Hz 

        self.gyroDatay = (self.gyroDatay/128)  #128 was discovered through trial and error
        #print self.gyroDatay
        self.gyroDatay = -self.gyroDatay

        self.pitch = -math.atan2(linear_acceleration[0],linear_acceleration[2])

        self.angley = 0.9*(self.angley + math.radians(self.gyroDatay)) + 0.1 * self.pitch


        self.gyroDataz = self.gyroDataz+(gyro[2]/500) #for 500 Hz rate

        self.gyroDataz = self.gyroDataz/128  #128 was discovered through trial and error
        #self.gyroDataz = self.gyroDataz

        self.yaw = math.atan2(linear_acceleration[0],linear_acceleration[1])

        self.anglez = 1.0*(self.anglez + math.radians(self.gyroDataz)) #+ 0.01 * self.yaw

        orientation_quaternion = tf.transformations.quaternion_from_euler(self.anglex, self.angley, self.anglez)



if __name__ == '__main__':
    rospy.init_node('stim300_gyro')
    i = Interface()
    while(True):
        i.run()
