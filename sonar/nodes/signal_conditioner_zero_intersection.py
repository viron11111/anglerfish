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
from sonar.msg import Sensitivity

#1600, -1600, -2000 (mm)
#0.0, 9.119770766119473, -9.016221156343818, -9.016221156343818

class condition():     

    def change_sensitivity(self, data):
        sens_rate = data.sensitivity
        if sens_rate == -1:
            self.break_val -= 0.01
            if self.break_val < self.min_break_val:
                self.break_val = self.min_break_val
        elif sens_rate == 1:
            self.break_val += 0.02
            if self.break_val > self.max_break_val:
                self.break_val = self.max_break_val  
        print self.break_val        

    def refine_data(self, channels, samples, sample_rate, data):  
        self.signal = []        

        #Seperate list into individual channels
        for i in range(channels):
            self.signal.append([])
            self.signal[i] = data[i::4]

        self.break_num = 0

        #determine average noise offset and apply offset
        avg_offset = [0]*channels
        for b in range(channels):
            #use first 100 samples
            l = self.signal[b][:200]
            avg_offset[b] = sum(l) / float(len(l))
            self.signal[b] = [x-avg_offset[b] for x in self.signal[b]]

        #find the first signal
        #looks for first signal to go above self.break_val value
        self.break_num = [0]*channels
        #print self.break_val
        for b in range(channels):
            for i in range(samples/4):
                if self.signal[b][i] >= self.break_val:
                    #print self.signal[b][i]
                    self.break_num[b] = i
                    #print break_num[b]
                    break        
        
        self.earliest_break_num = min(self.break_num)

        #rejection statement for samples triggered in middle of transmission (no start point)
        #print earliest_break_num
        if self.earliest_break_num > 200:

            #Buffering holder (zeros) based on the time of 1 period at 25 kHz
            num_samples_save = int((6.0/25000.0)*sample_rate)
            #num_samples_save = int((1.0/25000.0)*sample_rate)
            zeros = [0]*(num_samples_save/4)

            #eliminate all information before first signal by adding zeros in front of signal
            #appy to other 3 signals
            for b in range(channels):
                self.signal[b] = self.signal[b][self.earliest_break_num-num_samples_save::]#::]
                #self.signal[b] = np.append(zeros,self.signal[b])
                
            for b in range(channels):
                #print "break_num: ",break_num[b]
                #print "earliest_break_num ", earliest_break_num
                for i in range(len(self.signal[b])):
                    if i < self.break_num[b]-self.earliest_break_num: 
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
            num_samples_save = int((2.0/25000.0)*sample_rate)


            last_intersection = [0]*channels

            for b in range(channels):
                for i in range(final_length):
                    if self.signal[b][i] >= self.break_val:
                        trigger = i
                        #print "trigger: %i" % trigger
                        old1 = self.signal[b][trigger]
                        break
                for i in range(final_length-trigger):
                    first = int(i+trigger)
                    new1 = self.signal[b][i+trigger]
                    if old1 > 0 and new1 < 0:
                        #print "intersection1: %i" % (first)
                        old0 = first
                        break
                    else:
                        old1 = new1

                #range(100,-1,-1)                        
                for i in range(first, -1, -1):
                    zero = int(i)
                    new0 = self.signal[b][i+first]
                    if old0 > 0 and new0 < 0:
                        print "zero: %i" % zero
                        print "first: %i" % first
                        print "tricky difference[%i]: %i" % (b,zero-first)
                        #old3 = second
                        break
                    else:
                        old0 = new0
                        
                for i in range(first, -1, -1):
                    zero = int(i)
                    new0 = self.signal[b][i+first]
                    if old0 > 0 and new0 < 0:
                        print "zero: %i" % zero
                        print "first: %i" % first
                        print "tricky difference[%i]: %i" % (b,zero-first)
                        #old3 = second
                        break
                    else:
                        old0 = new0                        


                for i in range(final_length-first):
                    second = int(i+first)
                    new2 = self.signal[b][i+first]
                    if old2 < 0 and new2 > 0:
                        #print "intersection2: %i" % (second)
                        old3 = second
                        break
                    else:
                        old2 = new2

                for i in range(final_length-second):
                    third = int(i+second)
                    new3 = self.signal[b][i+second]
                    if old3 > 0 and new3 < 0:
                        #print "intersection3: %i" % (third)
                        last_intersection[b] = third
                        break
                    else:
                        old3 = new3       

            #print last_intersection        
            timediffs = [0]*channels

            for b in range(channels):
                timediffs[b] = (last_intersection[0]-last_intersection[b])*(1.0/2.0)

            print timediffs

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

            ######################
            self.signal[0] = [x*(amplitude_ratio[0]*0.5) for x in self.signal[0]]


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

            #for i in range(4):
            #    self.signal[i] = self.signal[i][:1750:]    

            #for i in range(channels):
                #print(max(self.signal[i]))
            #    print len(self.signal[i])
            #print"***"        


    def condition_data(self, msg):
        channels = msg.channels
        samples  = msg.samples
        sample_rate = msg.sample_rate
        adc_bit = msg.adc_bit
        data = msg.data      

        self.refine_data(channels, samples, sample_rate, data)

        error = 0

        #check to make sure signal sensitivity is not too low
        for i in range(channels):
            if len(self.signal[i]) < 1100 and error !=1:
                if max(self.signal[i]) < 8.0*self.break_val:
                    self.break_val += 0.01

                    print self.break_val

                    if self.break_val >= self.max_break_val:
                        self.break_val = self.max_break_val   
                        i = 3                                                    
                        self.counter = 10

                    self.counter += 1

                    if self.counter < 10:                        
                        self.refine_data(channels, samples, sample_rate, data)

            else:
                rospy.logwarn("Bad signal on channel %i" % i)
                error = 1
                break

        if self.counter >= 9:
            rospy.logerr("counter filled")
            self.break_val = 0.07
            error = 1
            self.counter = 0            

        if error != 1:

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
            hydro = '*you should not see this*'
            #rospy.logwarn("Missed beginning of signal, triggered late")
            #ospy.logwarn("Interference or weak channel")
            for i in range(4):
                if self.break_num[i] == self.earliest_break_num:
                    if i == 0:
                        hydro = 'A'
                    elif i == 1:
                        hydro = 'B'
                    elif i == 2:
                        hydro = 'C'
                    elif i == 3:
                        hydro = 'D'
                    rospy.logerr("Problem with hydrophone %c.  break_num = %s." % (hydro, self.break_num))


    def __init__(self):
        rospy.init_node('signal_conditioner')

        #rospy.Subscriber('hydrophones/pingmsg', Pingdata, self.condition_data) # for simulation and bags
        rospy.Subscriber('hydrophones/pingraw', Pingdata, self.condition_data)
        rospy.Subscriber('hydrophones/sensitivity', Sensitivity, self.change_sensitivity)

        self.simulate_pub = rospy.Publisher('hydrophones/pingconditioned', Pingdata, queue_size = 1)

        self.break_val = 0.05 #0.15 #voltage in which threshold is triggered
        self.min_break_val = -self.break_val

        self.max_break_val = 0.25
        self.min_break_val = 0.02
        self.counter = 0

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
