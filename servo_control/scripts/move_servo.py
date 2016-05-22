#!/usr/bin/python

from Adafruit_PWM_Servo_Driver import PWM
import time
import rospy

from std_msgs.msg import Int16

# Initialise the PWM device using the default address
pwm = PWM(0x40)
# Note if you'd like more debug output you can instead run:
#pwm = PWM(0x40, debug=True)

pwm.setPWMFreq(60)                        # Set frequency to 60 Hz

def setServoPulse(channel, pulse):
  pulseLength = 1000000                   # 1,000,000 us per second
  pulseLength /= 60                       # 60 Hz
  print "%d us per period" % pulseLength
  pulseLength /= 4096                     # 12 bits of resolution
  print "%d us per bit" % pulseLength
  pulse *= 1000
  pulse /= pulseLength
  pwm.setPWM(channel, 0, pulse)

def moving(pos):
    if pos.data < 140:
	pos.data = 140
    elif pos.data > 710:
	pos.data = 710

    pwm.setPWM(0, 0, pos.data)
    print pos    

def servo_move():
    rospy.init_node('move_servo', anonymous=True)
    rospy.Subscriber("servo_position", Int16, moving)

    rospy.spin()

if __name__ == '__main__':
	servo_move()
