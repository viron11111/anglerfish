#!/usr/bin/env python
import rospy
import rosparam
import random
import numpy as np
import matplotlib.pyplot as plt

from std_msgs.msg import Header
from pinger_tracker.msg import *

from advantech_pci1714.msg import *
#from multilateration import Multilaterator, ReceiverArraySim, Pulse

import sys
import math

from dynamic_reconfigure.server import Server
from pinger_tracker.cfg import SignalConfig
from pinger_tracker.srv import *

#1600, -1600, -2000 (mm)
#0.0, 9.119770766119473, -9.016221156343818, -9.016221156343818

class condition():     

    def condition_data(self, msg):
        channels = msg.channels
        samples  = msg.samples
        sample_rate = msg.sample_rate
        adc_bit = msg.adc_bit
        data = msg.data      

        self.signal = []        

        #Seperate list into individual channels
        for i in range(channels):
            self.signal.append([])
            self.signal[i] = data[i::4]

        break_num = 0

        #determine average noise offset and apply offset
        avg_offset = [0]*channels
        for b in range(channels):
            #use first 100 samples
            l = self.signal[b][:200]
            avg_offset[b] = sum(l) / float(len(l))
            self.signal[b] = [x-avg_offset[b] for x in self.signal[b]]

        #find the first signal
        #looks for first signal to go above self.break_val value
        break_num = [0]*channels
        for b in range(channels):
            for i in range(samples/4):
                if self.signal[b][i] >= self.break_val:
                    break_num[b] = i
                    break
        
        earliest_break_num = min(break_num)

        #rejection statement for samples triggered in middle of transmission (no start point)
        if earliest_break_num > 100:

            #Buffering holder (zeros) based on the time of 1 period at 25 kHz
            num_samples_save = int((7.0/25000.0)*sample_rate)
            zeros = [0]*num_samples_save        

            #eliminate all information before first signal by adding zeros in front of signal
            #appy to other 3 signals
            for b in range(channels):
                self.signal[b] = self.signal[b][earliest_break_num-num_samples_save::]
                self.signal[b] = np.append(zeros,self.signal[b])
                
            for b in range(channels):
                #print "break_num: ",break_num[b]
                #print "earliest_break_num ", earliest_break_num
                for i in range(len(self.signal[b])):
                    if i < break_num[b]-earliest_break_num: 
                        self.signal[b][i] = 0
                    else: 
                        break                
                    

            '''for b in range(channels):
                for i in range(len(self.signal[b])):
                    if i < break_num[b]-num_samples_save: self.signal[b][i] = 0
                    else: break'''

            #holder for new signal length (still contains samples following initial signal)
            final_length = len(self.signal[0])

            lastest_signal = 0
            current_signal = 0

            #using same variable as above
            #Buffering holder for keeping X periods of actual signal at 25 kHz
            num_samples_save = int((1.5/25000.0)*sample_rate)


            min_amp = [0]*channels
            max_amp = [0]*channels

            #find max value and min value in list for normalization purposes
            #only use 3 periods of 25 kHz length
            for b in range(channels):
                for i in range(final_length):
                    if self.signal[b][i] >= self.break_val:
                        min_amp[b] = min(self.signal[b][:i+num_samples_save])
                        max_amp[b] = max(self.signal[b][:i+num_samples_save])
                        break        
            
            #find greatest amplitude difference
            amplitude = [x - y for x, y in zip(max_amp, min_amp)]
            max_amplitude = max(amplitude)

            #determine ratio to normalize signal
            amplitude_ratio = [0]*channels
            for i in range(channels):
                amplitude_ratio[i] = max_amplitude/amplitude[i]

            #print amplitude_ratio



            ######################
            self.signal[0] = [x*(amplitude_ratio[0]*0.5) for x in self.signal[0]]
            
            '''degree = 5
            window=degree*2-1  
            weight=np.array([1.0]*window)  
            weightGauss=[]  
            for i in range(window):  
                i=i-degree+1  
                frac=i/float(window)  
                gauss=1/(np.exp((4*(frac))**2))  
                weightGauss.append(gauss) 
            weight=np.array(weightGauss)*weight  
            smoothed=[0.0]*(len(self.signal[0])-window)
            for i in range(len(smoothed)):  
                smoothed[i]=sum(np.array(self.signal[0][i:i+window])*weight)/sum(weight) 
            self.signal[0] = smoothed'''


            #********* NORMALIZATION for weak signals **********
            for b in range(channels-1):
                if amplitude_ratio[b+1] > 5:
                    self.signal[b+1] = [x*(amplitude_ratio[b+1]*0.25) for x in self.signal[b+1]]
                    phoneno = b+1
                    rospy.logwarn("SIGNAL DESCREPANCY: weak signal no hydrophone %i. Applying normalization " % phoneno)

            #function to allow 3 periods length of signal to continue
            #after 3 periods (at 25 kHz), following values are "zero'd"
            max_signal_range = [0]*channels
            for b in range(channels):
                #print("start")                
                zeros = []
                for i in range(final_length):
                    if self.signal[b][i] >= self.break_val:
                        current_signal = i
                        max_signal_range[b] = current_signal
                        if current_signal > lastest_signal:
                            lastest_signal = current_signal
                        self.signal[b] = self.signal[b][:num_samples_save+i:]
                        difference = final_length - len(self.signal[b])
                        zeros = [0]*difference
                        self.signal[b] = np.append(self.signal[b],zeros)
                        break

            #Allow 50 zeros passed the latest signal, crop all additional zeros following
            for i in range(channels):
                self.signal[i]= self.signal[i][:lastest_signal+num_samples_save+50:]

            #combine four signals back into one array
            condition_data = []

            for i in range(len(self.signal[0])):
                condition_data = np.append(condition_data,self.signal[0][i])
                condition_data = np.append(condition_data,self.signal[1][i])
                condition_data = np.append(condition_data,self.signal[2][i])
                condition_data = np.append(condition_data,self.signal[3][i])

            self.simulate_pub.publish(Pingdata(
                header=Header(stamp=rospy.Time.now(),
                              frame_id='signal_conditioner'),
                channels=channels,
                samples=len(self.signal[0])*4,
                data=condition_data,
                adc_bit = 12,
                sample_rate=sample_rate))     

        else:
            rospy.logwarn("Missed beginning of signal, triggered late")
            rospy.logwarn("Interference or weak channel")


    def __init__(self):
        rospy.init_node('signal_conditioner')

        #rospy.Subscriber('hydrophones/pingmsg', Pingdata, self.condition_data) # for simulation and bags
        rospy.Subscriber('hydrophones/pingraw', Pingdata, self.condition_data)

        self.simulate_pub = rospy.Publisher('hydrophones/pingconditioned', Pingdata, queue_size = 1)

        self.break_val = 0.02 #voltage in which threshold is triggered

        rate = rospy.Rate(1)

        while not rospy.is_shutdown():

            rate.sleep()

def main():
    
    condition()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print "Shutting down"
        pass
    


if __name__ == '__main__':
    main()              

    '''

        zz = (x1/del1) if del1 != 0 else 0
        z = (x3/del3) if del3 != 0 else 0

        A2 = zz - z   # eqn (14)

        #print "A2: %f" % A2

        B2 = -y3/del3 if del3 != 0 else 0

        #print "B2: %f" % B2

        holder = (x3*x3 + y3*y3-del3*del3)/(2.0*del3) if del3 != 0 else 0
        holder2 = (x1*x1-del1*del1)/(2*del1) if del1 != 0 else 0

        D2 = holder - holder2 

        #print "D2: %f" % D2

        x =  (B1*D2-B2*D1)/(A1*B2-A2*B1) if (A1*B2-A2*B1) != 0 else 0  # eqn (15)
        y = -(A1*D2-A2*D1)/(A1*B2-A2*B1) if (A1*B2-A2*B1) != 0 else 0

        myx = x 
        myy = y        

        T1 = -4*del1*del1
        T2 =  4*(x1*x1-del1*del1)*myx*myx + 4*x1*(del1*del1-x1*x1)*myx + del1*del1*del1*del1 -2*del1*del1*x1*x1 -4*del1*del1*myy*myy + x1*x1*x1*x1

        zsquared = -T2/T1 if T1 != 0 else 0

        z = -math.sqrt(abs(zsquared))'''
