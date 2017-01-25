#!/usr/bin/env python

import rospy
import sys
#from arduino_thruster_driver.msg import ThrusterCmd
from std_msgs.msg import Int16, String, Float32, Bool
from sub8_msgs.msg import Thrust, ThrusterCmd
from std_srvs.srv import SetBool, SetBoolResponse
from ctypes import c_ushort 


class ThrusterDriver:

    def thrust_cb(self, msg):
        '''Callback for recieving thrust commands
        These messages contain a list of instructions, one for each thruster
        '''

        #rospy.logwarn(self.kill)
        for thrust_cmd in list(msg.thruster_commands):
            self.command_thruster(thrust_cmd.name, thrust_cmd.thrust)

    def command_thruster(self, name, force):
        if force > 0.90:
            force = 0.90
        elif force < -0.90:
            force = -0.90

        if self.kill == False: #kill command is false

            green = rospy.Publisher('Green_led', Bool, queue_size=1)
            green.publish(True) #turn on green light

            if name == 'TOP':
                self.thrust = int(force * (32767.0)) - 3078
                self.thrstr1.publish(self.thrust)
            elif name == 'FL':
                self.thrust = int(force * (32767.0))
                self.thrstr2.publish(self.thrust)
            elif name == 'ML':
                self.thrust = int(force * (32767.0)) + 1119
                self.thrstr3.publish(self.thrust)
            elif name == 'BL':
                self.thrust = int(force * (32767.0)) 
                self.thrstr4.publish(self.thrust)
            elif name == 'FR':
                self.thrust = int(force * (32767.0)) - 356
                self.thrstr5.publish(self.thrust)
            elif name == 'MR':
                self.thrust = int(force * (32767.0)) + 1119
                self.thrstr6.publish(self.thrust)
            elif name == 'BR':
                self.thrust = int(force * (32767.0)) - 358
                self.thrstr7.publish(self.thrust)
            elif name == 'BTM':
                self.thrust = int(force * (32767.0)) - 3078
                self.thrstr8.publish(self.thrust)

        elif self.kill == True: #Kill command has been received

                green = rospy.Publisher('Green_led', Bool, queue_size=1)
                green.publish(False)  #turn off green light

                self.thrstr1.publish(0)
                self.thrstr2.publish(0)
                self.thrstr3.publish(0)
                self.thrstr4.publish(0)
                self.thrstr5.publish(0)
                self.thrstr6.publish(0)
                self.thrstr7.publish(0)
                self.thrstr8.publish(0)

    def ROV_kill(self, kill_cmd):
        self.kill = kill_cmd.data
        #rospy.logwarn(self.kill)
        return SetBoolResponse(success = True, message = str(self.kill))

    def stop_motor():
        thrstr1 = rospy.Publisher('thruster_cmd1', Int16, queue_size=1)
        thrstr2 = rospy.Publisher('thruster_cmd2', Int16, queue_size=1)
        thrstr3 = rospy.Publisher('thruster_cmd3', Int16, queue_size=1)
        thrstr4 = rospy.Publisher('thruster_cmd4', Int16, queue_size=1)
        thrstr5 = rospy.Publisher('thruster_cmd5', Int16, queue_size=1)
        thrstr6 = rospy.Publisher('thruster_cmd6', Int16, queue_size=1)
        thrstr7 = rospy.Publisher('thruster_cmd7', Int16, queue_size=1)
        thrstr8 = rospy.Publisher('thruster_cmd8', Int16, queue_size=1)
        thrstr1.publish(0)
        thrstr2.publish(0)
        thrstr3.publish(0)
        thrstr4.publish(0)
        thrstr5.publish(0)
        thrstr6.publish(0)
        thrstr7.publish(0)
        thrstr8.publish(0)


    def __init__(self):
        #rospy.Subscriber('name', String, self.name_func, queue_size = 1)
        #rospy.Subscriber('thrust', Float32, self.thrust_func, queue_size = 1)
        self.thrust_sub = rospy.Subscriber('thrusters/thrust', Thrust, self.thrust_cb, queue_size=1)

        rospy.Service('rov_kill', SetBool, self.ROV_kill)
        self.kill = True

        self.thrstr1 = rospy.Publisher('thruster_cmd1', Int16, queue_size=1)
        self.thrstr2 = rospy.Publisher('thruster_cmd2', Int16, queue_size=1)
        self.thrstr3 = rospy.Publisher('thruster_cmd3', Int16, queue_size=1)
        self.thrstr4 = rospy.Publisher('thruster_cmd4', Int16, queue_size=1)
        self.thrstr5 = rospy.Publisher('thruster_cmd5', Int16, queue_size=1)
        self.thrstr6 = rospy.Publisher('thruster_cmd6', Int16, queue_size=1)
        self.thrstr7 = rospy.Publisher('thruster_cmd7', Int16, queue_size=1)
        self.thrstr8 = rospy.Publisher('thruster_cmd8', Int16, queue_size=1)

        rate = rospy.Rate(25)   
        self.thrust = 0.0

        while not rospy.is_shutdown():
            rate.sleep()

    rospy.on_shutdown(stop_motor)

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
