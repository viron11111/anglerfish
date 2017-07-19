#!/usr/bin/env python
import rospy
import rosparam
import random
import numpy as np
import matplotlib.pyplot as plt
import time

from std_msgs.msg import Header
from pinger_tracker.msg import *
from multilateration import Multilaterator

import time
import sys

class phaser(Multilaterator):

    def hydrophone_locations(self, data):
        self.hydro0 = [data.hydro0_xyz[0],data.hydro0_xyz[1],data.hydro0_xyz[2]]
        self.hydro1 = [data.hydro1_xyz[0],data.hydro1_xyz[1],data.hydro1_xyz[2]]
        self.hydro2 = [data.hydro2_xyz[0],data.hydro2_xyz[1],data.hydro2_xyz[2]]
        self.hydro3 = [data.hydro3_xyz[0],data.hydro3_xyz[1],data.hydro3_xyz[2]]

    def pinger_position(self, tstamps):
        hydrophone_locations = np.array([self.hydro0, self.hydro1, self.hydro2, self.hydro3])

        c = 1.484  # millimeters/microsecond
        #hydrophone_array = ReceiverArraySim(hydrophone_locations, c)
        sonar = Multilaterator(hydrophone_locations, c, 'bancroft')

        res_msg = sonar.get_pulse_location(np.array(tstamps))
        #print "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$"
        print res_msg
        
        #res = np.array([res_msg[0], res_msg[1], res_msg[2]])

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

    def actual(self, data):
        self.actual_stamps = data.actual_time_stamps

    def parse_ping(self, data):        
        self.bit = data.adc_bit
        self.sample_rate = data.sample_rate
        self.actual_position = data.actual_position
        Ts = 1.0/self.sample_rate
        signal_periods = 1.0/25000.0
        channel_length = len(data.data)/data.channels

        self.signal = []
        self.timestamps = []

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
        self.start = time.clock()
        #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% David's Code %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        #reg_signal = mlat.TimeSignal1d(samples=self.signal[0])
        #non_ref_signals = [mlat.TimeSignal1d(samples=signal, sampling_freq) for signal in self.signal[1:]]
        #dtoa = mlat.get_dtoas(ref_signal, non_ref_signals)
        #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        for i in range(data.channels):
            self.timestamps.append(self.determine_phase(self.signal[0], self.signal[i]))

        self.end = time.clock()

        #sys.stderr.write("\x1b[2J\x1b[H")
        #print type(self.actual_stamps)
        #print type(timestamps)
        #difference = list(self.actual_stamps) - timestamps
        microseconds = [1e6,1e6,1e6,1e6]
        print "*********************************"
        print ("{}time to perform timestamps (Sec): {}%0.3f{}\n".format(self.W,self.O,self.W) % (self.end-self.start))
        #[0.0, 3.3333333333333333e-06, 0.0, 1.9999999999999998e-05]
        calculated = [x * y for x, y in zip(self.timestamps,microseconds)]

        self.calc_stamps_pub.publish(Calculated_time_stamps(
            header=Header(stamp=rospy.Time.now(),
                          frame_id='phase_shift'),
            calculated_time_stamps=calculated))

        print "{}calculated timestamps (uSec):".format(self.W)

        print "\t" + str(calculated)
        print "actual timestamps (uSec):"
        print "\t" + str([x * y for x, y in zip(self.actual_stamps,microseconds)])
        print "difference (uSec):"
        difference = [x - y for x, y in zip(list(self.actual_stamps), self.timestamps)]
        difference = [x * y for x, y in zip(difference,microseconds)]
        print "\t" + str(difference)
        errors = sum(map(abs, difference))
        print "Absolute sum of errors (uSec): {}%0.3f{}".format('\033[43m',self.W) % errors
       #mult = Multilaterator()
        #******************** figure out how to transfer variables from Multilateration ********************
        #self.pinger_position(calculated)
        #*****************************************************************************************************
        #print 

        #print "Actual position:\n\t" + "x: " + str(self.actual_position[0]) + " y: " + str(self.actual_position[1]) \
                #+ " z: " + str(self.actual_position[2]) + " (mm){}\n".format(self.W)        
        #print "*********************************"

    def __init__(self):

        self.start = time.clock()
        self.end = time.clock()
        self.timestamps = []
        self.actual_stamps = []
        self.actual_position = [0,0,0]

        self.hydro0 = [0,     0,     0]
        self.hydro1 = [-25.4, 0,     0]
        self.hydro2 = [25.4,  0,     0]
        self.hydro3 = [0,     -25.4, 0]

        rospy.Subscriber('hydrophones/hydrophone_locations', Hydrophone_locations, self.hydrophone_locations)
        rospy.Subscriber('/hydrophones/actual_time_stamps', Actual_time_stamps, self.actual)
        self.calc_stamps_pub = rospy.Publisher('/hydrophones/calculated_time_stamps', Calculated_time_stamps, queue_size = 1)
        #self.ls_pub = rospy.Publisher('hydrophones/Ls_pos', LS_pos, queue_size = 1)

        self.W  = '\033[0m'  # white (normal)
        self.R  = '\033[31m' # red
        self.G  = '\033[32m' # green
        self.O  = '\033[43m' # orange
        self.B  = '\033[34m' # blue
        self.P  = '\033[35m' # purple        
        
        rospy.init_node('phase_shift')

        rospy.Subscriber('/hydrophones/ping', Ping, self.parse_ping)

        rate = rospy.Rate(1)  #rate of signals, 5 Hz for Anglerfish

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