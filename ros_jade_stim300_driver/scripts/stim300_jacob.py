import serial
import numpy as np
import rospy
from geometry_msgs.msg import Quaternion, Vector3
from sensor_msgs.msg import Imu
import binascii
import struct
#import sub8_ros_tools

class Interface(object):
    _baudrate = 921600
    _datagram_lengths = {
        chr(0x93): 40 - 2,  # Rate, acceleration, inclination
    }
    # Rate, acceleration, incliination

    def __init__(self):
        self.serial = serial.Serial('/dev/serial/by-id/usb-FTDI_USB-RS422_Cable_FTXV40FP-if00-port0', baudrate=self._baudrate)
        self.datagram_identifier = chr(0x93)
        self.imu_pub = rospy.Publisher('/imu/data_raw', Imu, queue_size=1)
        self.last_msg = None
        self.skipped_msgs = 0

    def read(self):
        return self.serial.read(64 * 12)

    def sync(self):
        char = None
        k = 0
        while char != self.datagram_identifier:
            k += 1
            char = self.serial.read(1)
        return k

    def read_datagram(self):
        msg = self.serial.read(
            self._datagram_lengths[self.datagram_identifier]
        )

        start = 0
        # gyr
        gyro = np.fromstring(
            b'\x00' + msg[start + 0:start + 3][::-1] +
            b'\x00' + msg[start + 3:start + 6][::-1] +
            b'\x00' + msg[start + 6:start + 9][::-1],
            dtype='<i'
        ).astype(np.float32) / (2 ** 14)

        self.last_msg = gyro

        start += 10

        # acc
        linear_acceleration = np.fromstring(
            b'\x00' + msg[start + 0:start + 3][::-1] +
            b'\x00' + msg[start + 3:start + 6][::-1] +
            b'\x00' + msg[start + 6:start + 9][::-1],
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

        imu_msg = Imu(
            header=sub8_ros_tools.make_header(),
            angular_velocity=Vector3(*gyro),
            linear_acceleration=Vector3(*linear_acceleration)
        )
        self.imu_pub.publish(imu_msg)

    def run(self):
        l = self.sync()
        if l != 38:
            return
        self.read_datagram()


if __name__ == '__main__':
    rospy.init_node('stim300')
    i = Interface()
    while(True):
        i.run()
