#!/usr/bin/python

from Adafruit_PWM_Servo_Driver import PWM
import time
import rospy

from std_msgs.msg import String

# ===========================================================================
# Example Code
# ===========================================================================

# Initialise the PWM device using the default address
pwm = PWM(0x40)
# Note if you'd like more debug output you can instead run:
#pwm = PWM(0x40, debug=True)

servoMin = 150  # Min pulse length out of 4096
servoMax = 600  # Max pulse length out of 4096
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

def servo_move():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(.05) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
	pwm.setPWM(0, 0, servoMin)
	time.sleep(.9)
	pwm.setPWM(0, 0, servoMax)
	time.sleep(.9)

        rate.sleep()

if __name__ == '__main__':
    try:
        servo_move()
    except rospy.ROSInterruptException:
        pass
