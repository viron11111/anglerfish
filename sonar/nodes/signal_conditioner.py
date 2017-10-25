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

        for i in range(channels):
            self.signal.append([])
            self.signal[i] = data[i::4]

        break_num = 0

        #find the first signal
        for i in range(samples/4):
            if self.signal[0][i] >= self.break_val:
                break_num = i
                break
            elif self.signal[1][i] >= self.break_val:
                break_num = i
                break
            elif self.signal[2][i] >= self.break_val:
                break_num = i
                break
            elif self.signal[3][i] >= self.break_val:
                break_num = i
                break

        print break_num

        num_samples_save = int((1.0/25000.0)*sample_rate)
        zeros = [0]*num_samples_save        

        #eliminate all information before first signal
        for i in range(channels):
            self.signal[i]= self.signal[i][break_num-num_samples_save::]
            self.signal[i] = np.append(zeros,self.signal[i])

        final_length = len(self.signal[0])

        lastest_signal = 0
        current_signal = 0

        num_samples_save = int((3.0/25000.0)*sample_rate)

        for b in range(channels):
            #print("start")
            zeros = []
            for i in range(final_length):
                if self.signal[b][i] >= self.break_val:
                    current_signal = i
                    if current_signal > lastest_signal:
                        lastest_signal = current_signal
                    self.signal[b] = self.signal[b][:num_samples_save+i:]
                    difference = final_length - len(self.signal[b])
                    zeros = [0]*difference
                    self.signal[b] = np.append(self.signal[b],zeros)
                    break

        #print lastest_signal

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
        


    def __init__(self):
        rospy.init_node('signal_conditioner')

        rospy.Subscriber('hydrophones/pingmsg', Pingdata, self.condition_data) # for simulation and bags
        rospy.Subscriber('hydrophones/pingraw', Pingdata, self.condition_data)

        self.simulate_pub = rospy.Publisher('hydrophones/pingconditioned', Pingdata, queue_size = 1)

        self.break_val = 0.1

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
