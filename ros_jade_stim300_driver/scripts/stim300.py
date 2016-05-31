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
        self.stim_pub = rospy.Publisher('stim', Float32, queue_size=1)
        #self.last_msg = Nonesens
        self.gyroData = 0
        self.angle = 0
        self.roll = 0

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

    def read_datagram(self):
        msg = self.serial.read(
            self._datagram_lengths[self.datagram_identifier]
        )
 

        start = 0
        
        # gyr
        gyro = np.fromstring(
            b'\x00' + self.msg[start + 0:start + 3][::-1] +
            b'\x00' + self.msg[start + 3:start + 6][::-1] +
            b'\x00' + self.msg[start + 6:start + 9][::-1],
            dtype='<i'
        ).astype(np.float32) / (2 ** 14)
        #print gyro

        self.last_msg = gyro

        start += 10
        # acc
        linear_acceleration = np.fromstring(
            b'\x00' + self.msg[start + 0:start + 3][::-1] +
            b'\x00' + self.msg[start + 3:start + 6][::-1] +
            b'\x00' + self.msg[start + 6:start + 9][::-1],
            dtype='<i'
        ).astype(np.float32) / (2 ** 19)
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

        self.gyroData = (self.gyroData+gyro[0]/500) #for 500 Hz rate

        self.gyroData = self.gyroData/128  #128 was discovered through trial and error

        self.roll = math.atan2(linear_acceleration[1],linear_acceleration[2])

        self.angle = 0.98*(self.angle + math.radians(self.gyroData)) + 0.02 * self.roll

        #print self.angle


        #print math.radians(self.angle)

        #self.gyroData = self.gyroData + gyro[0]
        #print self.gyroData
        #angle = 0.98(angle + gyroData)

        #angle = 0.98*(angle + gyro[0]*dt) + 0.02*linear_acceleration[0]


#header=sub8_ros_tools.make_header(),

        imu_msg = Imu(            
            header= Header(
                stamp=rospy.get_rostime()
            ),
            angular_velocity=Vector3(*gyro),
            linear_acceleration=Vector3(*linear_acceleration)
            #orientation=Quaternion(inclination)
        )
        self.imu_pub.publish(imu_msg)
        self.stim_pub.publish(self.angle)

        #print msg.encode("hex")

    def run(self):
        l = self.sync()
        l = 38
        if l != 38:
            return
        self.read_datagram()


if __name__ == '__main__':
    rospy.init_node('stim300_gyro')
    i = Interface()
    while(True):
        i.run()
