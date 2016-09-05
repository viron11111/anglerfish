#!/usr/bin/env python

import rospy
import sys
#from arduino_thruster_driver.msg import ThrusterCmd
from std_msgs.msg import Int16, String, Float32
from sub8_msgs.msg import Thrust, ThrusterCmd
from std_srvs.srv import SetBool


class ThrusterDriver:
    def thrust_cb(self, msg):
        '''Callback for recieving thrust commands
        These messages contain a list of instructions, one for each thruster
        '''
        for thrust_cmd in list(msg.thruster_commands):
            self.command_thruster(thrust_cmd.name, thrust_cmd.thrust)

    def command_thruster(self, name, force):

        if name == 'TOP':
            self.thrust = force * (32767.0) #- 836
            self.thrstr1.publish(self.thrust)
        elif name == 'FL':
            self.thrust = force * (32767.0)
            self.thrstr2.publish(self.thrust)
        elif name == 'BL':
            self.thrust = force * (32767.0) #+ 404
            self.thrstr3.publish(self.thrust)
        elif name == 'FR':
            self.thrust = force * (32767.0) #+ 386
            self.thrstr4.publish(self.thrust)
        elif name == 'BR':
            self.thrust = force * (32767.0)
            self.thrstr5.publish(self.thrust)
        elif name == 'BTM':
            self.thrust = force * (32767.0)
            self.thrstr6.publish(self.thrust)                                    

    def ROV_kill(self):
            print("hello")

    def __init__(self):
        #rospy.Subscriber('name', String, self.name_func, queue_size = 1)
        #rospy.Subscriber('thrust', Float32, self.thrust_func, queue_size = 1)
        self.thrust_sub = rospy.Subscriber('thrusters/thrust', Thrust, self.thrust_cb, queue_size=1)

	s = rospy.Service('rov_kill', SetBool, self.ROV_kill)
	#self.kill = rospy.ServiceProxy('rov_kill', ROV_kill)

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
