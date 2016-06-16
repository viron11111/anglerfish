#!/usr/bin/python

from Adafruit_PWM_Servo_Driver import PWM
import time
import rospy

from std_msgs.msg import Int16

class servo_control():

  def setServoPulse(channel, pulse):
    pulseLength = 1000000                   # 1,000,000 us per second
    pulseLength /= 60                       # 60 Hz
    pulseLength /= 4096                     # 12 bits of resolution
    pulse *= 1000
    pulse /= pulseLength
    self.pwm.setPWM(channel, 0, pulse)

  def moving(self, pos):
    if pos.data < 140:
      rospy.logwarn("Servo min/max values exceeded: %d.  Acceptable range: 140-710" % pos.data)
      pos.data = 140
    elif pos.data > 710:
      rospy.logwarn("Servo min/max values exceeded: %d.  Acceptable range: 140-710" % pos.data)
      pos.data = 710

    self.pwm.setPWM(0, 0, pos.data)    

  def center():
    pwm = PWM(0x40)
    pwm.setPWMFreq(60)
    pwm.setPWM(0, 0, 365)

  def __init__(self):
    rospy.Subscriber("servo_position", Int16, self.moving)
    self.pwm = PWM(0x40)
    self.pwm.setPWMFreq(60)                        # Set frequency to 60 Hz

  rospy.on_shutdown(center)

def main(args):
  rospy.init_node('move_servo', anonymous=False)

  servo_control()

  try:
    rospy.spin()
  except rospy.ROSInterruptException:
    print "Shutting down"
    pass

if __name__ == '__main__':
  main(sys.argv)