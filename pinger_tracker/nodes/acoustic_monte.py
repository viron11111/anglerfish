#!/usr/bin/env python
import rospy
import rosparam
import random
import numpy as np
import matplotlib.pyplot as plt

from std_msgs.msg import Header, Bool
from pinger_tracker.msg import *

import dynamic_reconfigure.client

#def callback(config):
    #rospy.loginfo("Config changed")

class monte(object):

    def actual_position(self, data):
        self.actualpostion = list(data.actual_position)
        print self.actualpostion

    def __init__(self):

        client = dynamic_reconfigure.client.Client("signal_simulator", timeout=30)#, config_callback=callback)
        
        self.toggle = rospy.Publisher('hydrophones/signal_trigger', Bool, queue_size = 1)
        rospy.Subscriber('hydrophones/ping', Ping, self.actual_position)

        rate = rospy.Rate(0.5)  #rate of signals, 5 Hz for Anglerfish

        client.update_configuration({"signal_gen_trigger":'True'})

        trigger = 0

        while not rospy.is_shutdown():

            rand_num = random.uniform(25,50)
            client.update_configuration({"signal_freq":rand_num})

            self.toggle.publish(Bool(
                    data=trigger))

            trigger = not trigger

            rate.sleep()        



def main():
    rospy.init_node('acoustic_monte', anonymous=False)

    rospy.loginfo("Waiting for signal_simulator service...")

    rospy.wait_for_service("/signal_simulator/set_parameters")

    rospy.loginfo("Beginning Monte")



    monte()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print "Shutting down"
        pass
    


if __name__ == '__main__':
    main()