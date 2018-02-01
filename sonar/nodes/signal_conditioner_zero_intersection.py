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
from sonar.msg import Sensitivity, Slope, Negative_slope

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
        #print self.break_val        

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


            #*********************************************************************
            #****************NORMALIZATION****************************************
            #*********************************************************************
            abs_signal = [[],[],[],[]]
            signal_average = [0]*channels
  
            for i in range(channels):
                abs_signal[i] = map(abs, self.signal[i])
                signal_average[i] = np.mean(abs_signal[i])

            #print signal_average
            max_signal_average = max(signal_average)

            for i in range(channels):
                self.signal[i] = [z * (max_signal_average/signal_average[i]) for z in self.signal[i]]

            #*********************************************************************
            #*********************************************************************
            #*********************************************************************          

            #*******************************************************
            #************FIND NOISE FLOOR AMPLITUDE*****************
            #*******************************************************
            abs_signal = [[],[],[],[]]
            signal_average = [0]*channels
            max_noise = [0]*channels

            max_sample_size = 1200
            min_sample_size = 400

            #print self.previous_noise_floor_position

            for i in range(channels):
                abs_signal[i] = map(abs, self.signal[i][:(self.previous_noise_floor_position[i]-40)])
                #signal_average[i] = np.mean(abs_signal[i])
                max_noise[i] = max(abs_signal[i]) + 0.05#0.03
                if max_noise[i] >= 0.25:
                    max_noise[i] = 0.25
                elif max_noise[i] <= 0.05:
                    max_noise[i] = 0.05                   

            #max_noise = [z - 0.03 for z in max_noise[z]]

            #print "max_noise: %s" % max_noise
            #*******************************************************
            #*******************************************************
            #*******************************************************

            old = self.signal[0][0]
            sample_list = []
            max_cut_length = [0]*channels

            #test_var = [[100,50],[101,25]]
            #print test_var[0][1]
            for b in range(channels):
                sample_list = []
                list_dif = []

                for i in range(final_length):
                    new = self.signal[b][i]
                    if (old < 0 and new > 0) or (old > 0 and new < 0):
                        sample_list = np.append(sample_list,i)
                        #print i
                    old = new
                #print sample_list

                list_dif = [0]*len(sample_list)

                for i in range(len(sample_list)-1):
                    list_dif[i] = sample_list[i] - sample_list[i+1] 

                #print list_dif

                for i in range(len(list_dif)):
                    
                    if i < len(list_dif)-5:
                        average = (list_dif[i]+list_dif[i+1]+list_dif[i+2]+list_dif[i+3]+list_dif[i+4])/5

                        if average < -32.0 and average > -35.0:
                            max_value = max(self.signal[b][int(sample_list[i]):int(sample_list[i+1])])
                            min_value = min(self.signal[b][int(sample_list[i]):int(sample_list[i+1])])
                            #print "max: = %f min = %f" % (max_value, min_value)
                            #print "list_dif: %f" % list_dif[i+4]
                            #print "sample_list: %i" % sample_list[i+4]
                            if max_value > max_noise[b]:
                                cut_point = int(sample_list[i+7])
                                self.previous_noise_floor_position[b]=int(sample_list[i])
                                if self.previous_noise_floor_position[b] >= max_sample_size:
                                    self.previous_noise_floor_position[b] = max_sample_size
                                elif self.previous_noise_floor_position[b] <= min_sample_size:
                                    self.previous_noise_floor_position[b] = min_sample_size
                                #max_cut_length[b] = cut_point
                                self.signal[b] = self.signal[b][:cut_point]
                                break
                            elif min_value < -max_noise[b]:
                                cut_point = int(sample_list[i+6])
                                self.previous_noise_floor_position[b]=int(sample_list[i])
                                if self.previous_noise_floor_position[b] >= max_sample_size:
                                    self.previous_noise_floor_position[b] = max_sample_size   
                                elif self.previous_noise_floor_position[b] <= min_sample_size:
                                    self.previous_noise_floor_position[b] = min_sample_size                                                                 
                                #max_cut_length[b] = cut_point
                                self.signal[b] = self.signal[b][:cut_point]
                                break
                    else:
                        rospy.logerr("Bad signal on channel %i" % b)

            lengths = [len(self.signal[0]),len(self.signal[1]),len(self.signal[2]),len(self.signal[3])]
            #print lengths

            max_samples = max(lengths)
            #max_samples = max(max_cut_length)

            #for i in range(channels):
            #    self.signal[i] = self.signal[i][:max_samples]

            #print "max_number of samples: %i" % max_samples

            possible_time_stamps = [0]*channels

            for i in range(channels):
                possible_time_stamps[i] = (lengths[0] - lengths[i])/2.0
            print possible_time_stamps

            for i in range(channels):
                difference = max_samples - lengths[i]
                zeros = [0]*(difference+50)
                self.signal[i] = np.append(self.signal[i], zeros)



            #*******************************************************
            #************SLOPE APPROACH*****************************
            #*******************************************************

            old = self.signal[0][0]
            sample_list = []
            positive_list = []
            max_value_list = []
            difference_list = []
            slope = []
            slope_ratio = []

            #Find the zero crossing, either negative or positive
            for i in range(len(self.signal[0])):
                new = self.signal[0][i]
                if (old < 0 and new > 0) or (old > 0 and new < 0):
                    sample_list = np.append(sample_list,i)
                    #print i
                old = new

            #Create a positive list of crossing, starts positive and crosses to negative value
            for i in range(len(sample_list)-1):
                max_unit = max(self.signal[0][int(sample_list[i]):int(sample_list[i+1])])
                if max_unit > 0:
                    positive_list = np.append(positive_list,sample_list[i])
                    max_value_list = np.append(max_value_list,max_unit)

            #placeholder for a slope value
            slope_old = 0.1

            #Store a list of slopes of i and i+1
            for i in range(len(max_value_list)-1):
                slope = np.append(slope, (max_value_list[i+1] - max_value_list[i])/(sample_list[i+1]-sample_list[i]))
                slope_new = slope[i]
                slope_ratio = np.append(slope_ratio,slope_new/slope_old)
                #print "sample: %f max_val: %0.2f slope: %0.4f ratio: %f" % (positive_list[i]/2000000, max_value_list[i], slope[i], slope_ratio[i])
                slope_old = slope_new



            for i in range(len(positive_list)-1):
                positive_list[i] = (positive_list[i]+sample_list[2*i+2])/2.0

            voltage = [0]*len(positive_list)

            for i in range(len(positive_list)):
                voltage[i] = self.signal[0][int(positive_list[i])]

            self.slope_pub.publish(Slope(
                header=Header(stamp=rospy.Time.now(),
                              frame_id='slope_values'),
                sample=positive_list,
                max_val=max_value_list,
                ratio=slope_ratio,
                slope=slope,
                voltage=voltage))

            old = self.signal[0][0]
            sample_list = []
            negative_list = []
            min_value_list = []
            difference_list = []
            slope = []
            slope_ratio = []

            for i in range(len(self.signal[0])):
                new = self.signal[0][i]
                if (old < 0 and new > 0) or (old > 0 and new < 0):
                    sample_list = np.append(sample_list,i)
                    #print i
                old = new

            for i in range(len(sample_list)-1):
                min_unit = min(self.signal[0][int(sample_list[i]):int(sample_list[i+1])])
                if min_unit < 0:
                    negative_list = np.append(negative_list,sample_list[i])
                    min_value_list = np.append(min_value_list,min_unit)

            slope_old = -0.1

            for i in range(len(min_value_list)-1):
                slope = np.append(slope, (min_value_list[i+1] - min_value_list[i])/(sample_list[i+1]-sample_list[i]))
                slope_new = slope[i]
                slope_ratio = np.append(slope_ratio,slope_new/slope_old)
                #print "sample: %f min_val: %0.2f slope: %0.4f ratio: %f" % (negative_list[i]/2000000, min_value_list[i], slope[i], slope_ratio[i])
                slope_old = slope_new


            for i in range(len(negative_list)-1):
                negative_list[i] = (negative_list[i]+sample_list[2*i+1])/2.0

            voltage = [0]*len(negative_list)

            for i in range(len(negative_list)):
                voltage[i] = self.signal[0][int(negative_list[i])]

            self.neg_slope_pub.publish(Negative_slope(
                header=Header(stamp=rospy.Time.now(),
                              frame_id='negative_slope_values'),
                sample=negative_list,
                max_val=min_value_list,
                ratio=slope_ratio,
                slope=slope,
                voltage=voltage))            

            #*******************************************************
            #*******************************************************
            #*******************************************************  






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

                    #print self.break_val

                    if self.break_val >= self.max_break_val:
                        self.break_val = self.max_break_val   
                        i = 3                                                    
                        self.counter = 10

                    self.counter += 1

                    #if self.counter < 10:                        
                     #   self.refine_data(channels, samples, sample_rate, data)

            else:
                #rospy.logwarn("Bad signal on channel %i" % i)
                error = 1
                break

        if self.counter >= 9:
            #rospy.logerr("counter filled")
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

            '''self.simulate_pub.publish(Pingdata(
                header=Header(stamp=rospy.Time.now(),
                              frame_id='signal_conditioner'),
                channels=channels,
                samples=len(self.signal[0])*4,
                data=condition_data,
                adc_bit = 12,
                sample_rate=sample_rate))   '''  

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
                    #rospy.logerr("Problem with hydrophone %c.  break_num = %s." % (hydro, self.break_num))


    def __init__(self):
        rospy.init_node('signal_conditioner')

        #rospy.Subscriber('hydrophones/pingmsg', Pingdata, self.condition_data) # for simulation and bags
        rospy.Subscriber('hydrophones/pingraw', Pingdata, self.condition_data)
        rospy.Subscriber('hydrophones/sensitivity', Sensitivity, self.change_sensitivity)

        self.simulate_pub = rospy.Publisher('hydrophones/pingconditioned', Pingdata, queue_size = 1)
        self.slope_pub = rospy.Publisher('hydrophones/slope', Slope, queue_size = 1)
        self.neg_slope_pub = rospy.Publisher('hydrophones/negative_slope', Negative_slope, queue_size=1)

        self.break_val = 0.05 #0.15 #voltage in which threshold is triggered
        self.min_break_val = -self.break_val

        self.max_break_val = 0.25
        self.min_break_val = 0.02
        self.counter = 0

        self.previous_noise_floor_position = [400,400,400,400]

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
