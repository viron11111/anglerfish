#!/usr/bin/env python
import rospy
import rosparam
import random
import numpy as np
import matplotlib.pyplot as plt
import time

from std_msgs.msg import Header
from pinger_tracker.msg import *
from multilateration import Multilaterator, ReceiverArraySim, Pulse

class phaser():

    def determine_phase(self, ref_sig, a_sig):
        channel_length = len(ref_sig)
        zeros = [0]*channel_length
        signed = [-(2**self.bit)/2]*channel_length
        ref_sig = list(ref_sig)
        ref_sig = [x + y for x, y in zip(ref_sig, signed)]
        a_sig = list(a_sig)
        a_sig = [x + y for x, y in zip(a_sig, signed)]        

        reference = ref_sig + zeros
        #print signed
        signal = zeros + a_sig
        
        sum_val = 0
        sum_val_max = 0
        phase_holder = 0
        max_list = []


        #*********************No aligning toward middle, Stopped HERE***********
        for z in range(2*channel_length):
            #summing function
            sum_val = sum([x * y for x, y in zip(reference,signal)])

            if sum_val >=  sum_val_max:
                sum_val_max = sum_val
                phase_holder = z
                #print reference
                max_list = reference

            reference.insert(0, reference.pop(2*channel_length-1))
            signal.insert(2*channel_length-1, signal.pop(0))

        #print sum_val_max
        print max_list
        shift = (phase_holder - (channel_length/2))
        #************************Stopped HERE*******************************

    def parse_ping(self, data):
        self.bit = data.adc_bit
        channel_length = len(data.data)/4
        self.ref_signal = [None]*(channel_length)
        self.ref_signal = data.data[::4]
        self.a_signal = [None]*(channel_length)
        self.a_signal = data.data[1::4]
        self.determine_phase(self.ref_signal, self.a_signal)

    def __init__(self):
        rospy.init_node('phase_shift')

        rospy.Subscriber('/hydrophones/ping', Ping, self.parse_ping)

        rate = rospy.Rate(20)  #rate of signals, 5 Hz for Anglerfish

        while not rospy.is_shutdown():

        	rate.sleep()


def main():
    rospy.init_node('phase_shift', anonymous=False)

    phaser()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print "Shutting down"
        pass
    


if __name__ == '__main__':
    main()