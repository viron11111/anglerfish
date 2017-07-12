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
        #print channel_length
        zeros = [0]*channel_length
        signed = [-(2**self.bit)/2]*channel_length
        ref_sig = list(ref_sig)
        ref_sig = [x + y for x, y in zip(ref_sig, signed)]
        a_sig = list(a_sig)
        a_sig = [x + y for x, y in zip(a_sig, signed)]        

        signal = zeros + a_sig
        reference = zeros[:channel_length/2] + ref_sig + zeros[:channel_length/2]        
        
        sum_val = 0
        sum_val_max = 0
        phase_holder = 0
        max_list = []

        for z in range(2*channel_length):
            
            #summing function
            sum_val = sum([x * y for x, y in zip(reference,signal)])

            if sum_val >=  sum_val_max:
                sum_val_max = sum_val
                phase_holder = z
                max_list = signal

            signal.insert(2*channel_length-1, signal.pop(0))

        #print float(channel_length/2-phase_holder)
        return (channel_length/2-phase_holder)*(1.0/self.sample_rate)


    def parse_ping(self, data):
        self.bit = data.adc_bit
        self.sample_rate = data.sample_rate
        Ts = 1.0/self.sample_rate
        signal_periods = 1.0/25000.0
        channel_length = len(data.data)/data.channels

        self.signal = []
        timestamps = []

        for i in range(data.channels):
            self.signal.append([])
            self.signal[i] = data.data[i::4]
            left_periods = int((channel_length/2)-signal_periods/Ts)
            right_periods = int((channel_length/2)+signal_periods/Ts)
            #print self.signal
            self.signal[left_periods:right_periods]

        #self.ref_signal = [None]*(channel_length)
        #self.ref_signal = data.data[::4]
        #self.a_signal = [None]*(channel_length)
        #self.a_signal = data.data[1::4]
        
        #print "***"
        for i in range(data.channels):
            timestamps.append(self.determine_phase(self.signal[0], self.signal[i]))


        #[0.0, 3.3333333333333333e-06, 0.0, 1.9999999999999998e-05]
        print timestamps

    def __init__(self):
        rospy.init_node('phase_shift')

        rospy.Subscriber('/hydrophones/ping', Ping, self.parse_ping)

        rate = rospy.Rate(100)  #rate of signals, 5 Hz for Anglerfish

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