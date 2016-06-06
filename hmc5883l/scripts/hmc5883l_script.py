#!/usr/bin/python

import sys
import smbus
import time
import math
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import MagneticField

def read_byte(adr):
    return bus.read_byte_data(address, adr)

def read_word(adr):
    high = bus.read_byte_data(address, adr)
    low = bus.read_byte_data(address, adr+1)
    val = (high << 8) + low
    return val

def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def write_byte(adr, value):
    bus.write_byte_data(address, adr, value)



class measure_headings():

	def read_byte(adr):
    	return bus.read_byte_data(address, adr)

	def read_word(adr):
	    high = bus.read_byte_data(address, adr)
	    low = bus.read_byte_data(address, adr+1)
	    val = (high << 8) + low
	    return val
	    
	def read_word_2c(adr):
	    val = read_word(adr)
	    if (val >= 0x8000):
	        return -((65535 - val) + 1)
	    else:
	        return val

	def write_byte(adr, value):
	    bus.write_byte_data(address, adr, value)

	def __init__(self):
		self.bus = smbus.SMBus(1)

		self.HMC5883L_ADDR       = 0x1E  #HMC5883L address

		self.mag_pub = rospy.Publisher("magnetic_headings", MagneticField, queue_size=1)

		self.write_byte(0x00, 0b01110000) # Set to 8 samples @ 15Hz
		self.write_byte(0x01, 0b00100000) # 1.3 gain LSb / Gauss 1090 (default)
		self.write_byte(0x02, 0b00000000) # Continuous sampling

	def get_reading(self):



def main(args):
	rospy.init_node('hmc5883l_magnetometer', anonymous=False)

	measure_headings()

        try:
		rospy.spin()
        except rospy.ROSInterruptException:
		print "Shutting down"
                pass
	


if __name__ == '__main__':
	main(sys.argv)