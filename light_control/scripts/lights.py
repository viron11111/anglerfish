#!/usr/bin/python

#import RPi.GPIO as GPIO
import pigpio
import time
import rospy
from std_msgs.msg import Bool, String

class start_lights():

	def green(self, power):
		if power.data == 1:
			self.pi.write(16, 1)
		else:
			self.pi.write(16, 0)

	def white(self, power):
	    if power.data == 1:
	        self.pi.write(12, 1)
	    else:
	        self.pi.write(12, 0)

	def lights_off(self):
		self.pi.write(12, 0)
		self.pi.write(16, 0)		

	def __init__(self):
		rospy.Subscriber('Green_led', Bool, self.green)
		rospy.Subscriber('White_led', Bool, self.white)
		self.green_pub = rospy.Publisher("green_light", String, queue_size=1)
		self.white_pub = rospy.Publisher("white_light", String, queue_size=1)

		self.pi = pigpio.pi()
		self.pi.set_mode(12, pigpio.OUTPUT)
		self.pi.set_mode(16, pigpio.OUTPUT)

		rate = rospy.Rate(10) #Hz

		while not rospy.is_shutdown():

		    rate.sleep()	

    rospy.on_shutdown(self.lights_off)



rospy.on_shutdown(lights_off)

if __name__ == '__main__':
        rospy.init_node('LED_control', anonymous=False)

        start_lights()

        try:
                rospy.spin()
        except rospy.ROSInterruptException:
                print "Shutting down"
                pass


