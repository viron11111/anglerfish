#!/usr/bin/env python

import rospy
import sys
#from arduino_thruster_driver.msg import ThrusterCmd
from std_msgs.msg import Int16, String, Float32

class ThrusterDriver:
    def name_func(self, name):

        if name.data == 'TOP':
            self.thrstr1.publish(self.thrust)
        elif name.data == 'FL':
            self.thrstr2.publish(self.thrust)
        elif name.data == 'BL':
            self.thrstr3.publish(self.thrust)
        elif name.data == 'FR':
            self.thrstr4.publish(self.thrust)
        elif name.data == 'BR':
            self.thrstr5.publish(self.thrust)
        elif name.data == 'BTM':
            self.thrstr6.publish(self.thrust)                                    

    def thrust_func(self, force):
        self.thrust = force.data * (32767)

    def __init__(self):
        rospy.Subscriber('name', String, self.name_func, queue_size = 1)
        rospy.Subscriber('thrust', Float32, self.thrust_func, queue_size = 1)

        self.thrstr1 = rospy.Publisher('thruster_cmd1', Int16, queue_size=1)
        self.thrstr2 = rospy.Publisher('thruster_cmd2', Int16, queue_size=1)
        self.thrstr3 = rospy.Publisher('thruster_cmd3', Int16, queue_size=1)
        self.thrstr4 = rospy.Publisher('thruster_cmd4', Int16, queue_size=1)
        self.thrstr5 = rospy.Publisher('thruster_cmd5', Int16, queue_size=1)
        self.thrstr6 = rospy.Publisher('thruster_cmd6', Int16, queue_size=1)

        rate = rospy.Rate(20)   
        self.thrust = 0.0

        while not rospy.is_shutdown():
            rate.sleep()

def main(args):
    rospy.init_node('t100_thruster_driver', anonymous=False)

    ThrusterDriver()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print "Shutting down"
        pass
    


if __name__ == '__main__':
    main(sys.argv)