#!/usr/bin/python

import RPi.GPIO as GPIO
import time
import rospy
from std_msgs.msg import Bool  


def green(power):
	if power.data == 1:
		GPIO.output(16, 1)
	else:
		GPIO.output(16, 0)

def white(power):
        if power.data == 1:
	        GPIO.output(12, 1)
        else:
                GPIO.output(12, 0)

def lights_off():
	GPIO.output(12, 0)
	GPIO.output(16, 0)
	GPIO.cleanup()		

def start_lights():
	rospy.Subscriber('Green_led', Bool, green)
	rospy.Subscriber('White_led', Bool, white)

	GPIO.setmode(GPIO.BOARD)
	#GPIO.setmode(GPIO.BCM)
	GPIO.setwarnings(False)

	GPIO.setup(12, GPIO.OUT) #white light
	GPIO.setup(16, GPIO.OUT) #green light
		
rospy.on_shutdown(lights_off)

if __name__ == '__main__':
        rospy.init_node('LED_control', anonymous=False)

        start_lights()

        try:
                rospy.spin()
        except rospy.ROSInterruptException:
                print "Shutting down"
                pass


