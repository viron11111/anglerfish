#!/usr/bin/python

import time
import rospy
import random

from std_msgs.msg import Int16

servoMin = 150  # Min pulse length out of 4096
servoMax = 600  # Max pulse length out of 4096

def servo_move():
    pub = rospy.Publisher('servo_position', Int16, queue_size=10)
    rospy.init_node('desired_postion', anonymous=True)
    rate = rospy.Rate(1) #hz
    while not rospy.is_shutdown():
        position = random.randint(140, 710)

	rospy.loginfo(position)
        pub.publish(position)
        rate.sleep()

if __name__ == '__main__':
    try:
        servo_move()
    except rospy.ROSInterruptException:
        pass
