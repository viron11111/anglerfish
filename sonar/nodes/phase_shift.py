#!/usr/bin/env python
import rospy
import rosparam
import random
import numpy as np
import matplotlib.pyplot as plt
import time

from std_msgs.msg import Header

from advantech_pci1714.msg import *

from pinger_tracker.msg import Calculated_time_stamps
import matplotlib.pyplot as plt

#from multilateration import Multilaterator
#import multilateration as mlat
#from time_signal_1d import TimeSignal1D

import time
import sys

class phaser():

    '''def hydrophone_locations(self, data):
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

    def a_add_zeros(self, a_sig):
        channel_length = len(a_sig)

        if channel_length % 2 != 0:
            zeros = [0]*(channel_length+1)
        else:
            zeros = [0]*(channel_length)      

        signed = [-(2**self.bit)/2]*channel_length

        a_sig = list(a_sig)
        a_sig = [x + y for x, y in zip(a_sig, signed)]     

        signal = zeros + a_sig

        return signal        


    def ref_add_zeros(self, ref_sig):
        channel_length = len(ref_sig)

        if channel_length % 2 != 0:
            zeros = [0]*(channel_length+1)
        else:
            zeros = [0]*(channel_length)      

        signed = [-(2**self.bit)/2]*channel_length

        ref_sig = list(ref_sig)
        ref_sig = [x + y for x, y in zip(ref_sig, signed)]   
        
        reference = zeros[channel_length/2:] + ref_sig + zeros[:channel_length/2+1]           

        return reference

    def a_add_zeros_5v(self, a_sig):
        channel_length = len(a_sig)

        if channel_length % 2 != 0:
            zeros = [0]*(channel_length+1)
        else:
            zeros = [0]*(channel_length)      

        signed = [-(2**self.bit)/2]*channel_length

        a_sig = list(a_sig)
        #a_sig = [x + y for x, y in zip(a_sig, signed)]     

        signal = zeros + a_sig

        return signal        


    def ref_add_zeros_5v(self, ref_sig):
        channel_length = len(ref_sig)

        if channel_length % 2 != 0:
            zeros = [0]*(channel_length+1)
        else:
            zeros = [0]*(channel_length)      

        signed = [-(2**self.bit)/2]*channel_length

        ref_sig = list(ref_sig)
        #ref_sig = [x + y for x, y in zip(ref_sig, signed)]   
        
        reference = zeros[channel_length/2:] + ref_sig + zeros[:channel_length/2+1]           

        return reference        



    def actual(self, data):
        self.actual_stamps = data.actual_time_stamps




    def calculate_time_stamps_phase(self,input):
        #ping = rospy.ServiceProxy('/hydrophones/ping', Ping_service)
        ping = rospy.ServiceProxy('/hydrophones/ping', Ping)
        data = ping()



        self.start = time.clock()

        self.timestamps = [0.0]

        #print self.ping_stamps

        for i in range(3):
            self.timestamps.append(self.determine_phase(self.signal[0], self.signal[i+1]))            

        self.end = time.clock()

        microseconds = [1e6,1e6,1e6,1e6]
        print "*********************************"
        print ("{}time to perform timestamps (Sec): {}%0.3f{}\n".format(self.W,self.O,self.W) % (self.end-self.start))
        
        calculated = [x * y for x, y in zip(self.timestamps,microseconds)]

        #for i in range(4):
        #    calculated[i] = calculated[i] + self.ping_stamps[i]

        print "{}calculated timestamps (uSec):".format(self.W)

        print "\t" + str(calculated)

        #astamps = rospy.ServiceProxy('/hydrophones/actual_time_stamps', Actual_time_stamps_service)
        #astamps = astamps()
        #self.actual_stamps = astamps.actual_time_stamps

        #print "actual timestamps (uSec):"
        #print "\t" + str(self.actual_stamps) #str([x * y for x, y in zip(self.actual_stamps,microseconds)])
        #print "difference (uSec):"
        
        #difference = [x - y for x, y in zip(list(self.actual_stamps), calculated)]
        
        #print "\t" + str(difference)
        #errors = sum(map(abs, difference))
        #print "Absolute sum of errors (uSec): {}%0.3f{}".format('\033[43m',self.W) % errors  

	self.calc_stamps_pub = rospy.Publisher('/hydrophones/calculated_time_stamps', Calculated_time_stamps, queue_size = 1)

        self.calc_stamps_pub.publish(Calculated_time_stamps(
            header=Header(stamp=rospy.Time.now(),
                          frame_id='phase_shift'),
            calculated_time_stamps=calculated))

        return Calculated_time_stamps_serviceResponse(calculated)'''
    def determine_phase(self, ref_sig, a_sig):
        channel_length = len(ref_sig)

        #signal = self.a_add_zeros(a_sig)    
        #reference = self.ref_add_zeros(ref_sig)
        #signal = self.a_add_zeros_5v(a_sig)    
        #reference = self.ref_add_zeros_5v(ref_sig)

        reference = ref_sig
        signal = a_sig


        length = len(reference)-1
      
        sum_val = 0
        sum_val_max = 0
        phase_holder = 0
        max_list = []

        #rospy.logwarn("REFERENCE")
        #rospy.loginfo(reference)
        #rospy.logwarn("SIGNAL")
        #rospy.loginfo(signal)

        cross_corr = np.correlate(reference, signal, mode='full')
        max_idx = cross_corr.argmax()

        #rospy.loginfo(len(cross_corr))
        #print len(reference)
        #plt.plot(cross_corr)
        #plt.ylabel('some numbers')
        #plt.show()


        
        phase_holder = len(reference) - 1 - max_idx
        #print phase_holder
        
        return phase_holder*(1.0/self.sample_rate)        

    def parse_ping(self, data):
        self.bit = data.adc_bit
        self.sample_rate = data.sample_rate
        #self.ping_stamps = data.stamps

        channels = data.channels
        Ts = 1.0/self.sample_rate
        signal_periods = 1.0/25000.0  #25k is the longest signal expected
        data = data.data[15000:21000:1]

        #channel_length = len(data.data)/data.channels
        #channel_length = len(data)/data.channels

        self.signal = []
        self.timestamps = [0.0]

        for i in range(channels):
            self.signal.append([])
            #self.signal[i] = data.data[i::4]
            self.signal[i] = data[i::4]
            #left_periods = int((channel_length/2)-signal_periods/Ts)
            #right_periods = int((channel_length/2)+signal_periods/Ts)
            
            #self.signal[left_periods:right_periods]
        #cross_corr = np.correlate(self.signal[0], self.signal[0], mode='full')
        #max_idx = cross_corr.argmax()
        #print max_idx            
        
        #print self.signal[0][0:10]
        for i in range(3):
            #self.timestamps.append(self.determine_phase(self.signal[0], self.signal[i+1]))
            self.timestamps.append(self.determine_phase(self.signal[0], self.signal[i+1]))

        microseconds = [1e6,1e6,1e6,1e6]
        calculated = [x * y for x, y in zip(self.timestamps,microseconds)]

        print ("%0.2f, %0.2f, %0.2f, %0.2f uSec" % (calculated[0], calculated[1], calculated[2], calculated[3]))

        self.calc_stamps_pub = rospy.Publisher('/hydrophones/calculated_time_stamps', Calculated_time_stamps, queue_size = 1)
        self.calc_stamps_pub.publish(Calculated_time_stamps(
            header=Header(stamp=rospy.Time.now(),
                          frame_id='phase_shift'),
            calculated_time_stamps=calculated))

    def __init__(self):

        rospy.init_node('phase_shift')

        #self.start = time.clock()
        #self.end = time.clock()
        #self.timestamps = []
        #self.actual_stamps = []
        #self.actual_position = [0,0,0]

        #self.hydro0 = [0,     0,     0]
        #self.hydro1 = [-25.4, 0,     0]
        #self.hydro2 = [25.4,  0,     0]
        #self.hydro3 = [0,     -25.4, 0]

        rospy.Subscriber('hydrophones/pingmsg', Pingdata, self.parse_ping)

        #rospy.Subscriber('/hydrophones/actual_time_stamps', Actual_time_stamps, self.actual)
        #rospy.Subscriber('/hydrophones/ping', Ping, self.parse_ping)

        self.calc_stamps_pub = rospy.Publisher('/hydrophones/calculated_time_stamps', Calculated_time_stamps, queue_size = 1)

        #rospy.Service('/hydrophones/calculated_time_stamps', Calculated_time_stamps_service, self.calculate_time_stamps_phase)

        self.W  = '\033[0m'  # white (normal)
        self.R  = '\033[31m' # red
        self.G  = '\033[32m' # green
        self.O  = '\033[43m' # orange
        self.B  = '\033[34m' # blue
        self.P  = '\033[35m' # purple  

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
