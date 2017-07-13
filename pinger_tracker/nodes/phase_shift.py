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

import time
import sys

class phaser():

    def determine_phase(self, ref_sig, a_sig):
        channel_length = len(ref_sig)
        #print len(ref_sig)
        #print len(a_sig)
        #print channel_length
        '''if channel_length % 2 != 0:
            #print "odd"
            ref_sig = ref_sig[1:channel_length+1]
            #print len(a_sig)
            a_sig = ref_sig[0:channel_length]
            #print len(a_sig)
            channel_length = len(ref_sig)'''

        if channel_length % 2 != 0:
            zeros = [0]*(channel_length+1)
            #print "odd"
        else:
            zeros = [0]*(channel_length)

        '''print"***"
        print channel_length
        print len(zeros[channel_length/2:])
        print len(zeros[:channel_length/2+1])
        print"***"'''
        signed = [-(2**self.bit)/2]*channel_length

        ref_sig = list(ref_sig)
        ref_sig = [x + y for x, y in zip(ref_sig, signed)]

        a_sig = list(a_sig)
        a_sig = [x + y for x, y in zip(a_sig, signed)]        

        #print len(a_sig)
        signal = zeros + a_sig
        #print len(signal)
        reference = zeros[channel_length/2:] + ref_sig + zeros[:channel_length/2+1]        
        #print len(reference)
        
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
        
        '''print "***"
        print channel_length
        print phase_holder
        print "***"'''

        #print float(channel_length/2-phase_holder)
        return (channel_length/2-phase_holder)*(1.0/self.sample_rate)


    def parse_ping(self, data):
        self.actual_stamps = data.actual_time_stamps
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
        start = time.clock()
        for i in range(data.channels):
            timestamps.append(self.determine_phase(self.signal[0], self.signal[i]))

        end = time.clock()

        #difference = self.actual_stamps - timestamps

        sys.stderr.write("\x1b[2J\x1b[H")
        #print type(self.actual_stamps)
        #print type(timestamps)
        #difference = list(self.actual_stamps) - timestamps
        microseconds = [1e6,1e6,1e6,1e6]
        
        print ("seconds to perform timestamps: {}%0.3f".format(self.O) % (end-start))
        #[0.0, 3.3333333333333333e-06, 0.0, 1.9999999999999998e-05]
        print "{}calculated timestamps:".format(self.W)
        print [x * y for x, y in zip(timestamps,microseconds)]
        print "actual timestamps:"
        print [x * y for x, y in zip(self.actual_stamps,microseconds)]
        print "difference:"
        difference = [x - y for x, y in zip(list(self.actual_stamps), timestamps)]
        difference = [x * y for x, y in zip(difference,microseconds)]
        print difference
        print "Absolute sum of errors (uSec)"
        errors = sum(map(abs, difference))
        print errors


    def __init__(self):
        self.W  = '\033[0m'  # white (normal)
        self.R  = '\033[31m' # red
        self.G  = '\033[32m' # green
        self.O  = '\033[43m' # orange
        self.B  = '\033[34m' # blue
        self.P  = '\033[35m' # purple
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