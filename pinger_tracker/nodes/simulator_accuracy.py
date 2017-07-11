#!/usr/bin/env python
import rospy
import rosparam
import random
import numpy as np
import matplotlib.pyplot as plt

from std_msgs.msg import Header
from pinger_tracker.msg import *
from multilateration import Multilaterator, ReceiverArraySim, Pulse

class accuracy():

    def __init__(self):
        rospy.init_node('simulator_accuracy')

        self.frame = rospy.get_param('~frame', '/transmitter')

        self.pos_pub = rospy.Publisher('hydrophones/simulated_position', Transmitter_position, queue_size = 1)

        rate = rospy.Rate(1)  #rate of signals, 5 Hz for Anglerfish

        while not rospy.is_shutdown():
        	self.pos_pub.publish(Transmitter_position(
                header=Header(stamp=rospy.Time.now(),
                              frame_id=self.frame),
                x_pos=int(np.random.normal(-1000,1000)),
                y_pos=int(np.random.normal(-1000,1000)),
                z_pos=int(np.random.normal(-1000,1000))))
        	rate.sleep()




def main():
	rospy.init_node('simulator_accuracy', anonymous=False)

	accuracy()

	try:
	    rospy.spin()
	except rospy.ROSInterruptException:
	    print "Shutting down"
	    pass

if __name__ == '__main__':
    main()