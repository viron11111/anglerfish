#!/usr/bin/env python
import rospy
import rosparam

import numpy as np
from scipy import optimize

#from sub8_msgs.srv import Sonar, SonarResponse

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from std_msgs.msg import UInt16
#from pinger_tracker.msg import *

from advantech_pci1714.srv import *
from advantech_pci1714.msg import *

from hydrophones.msg import Ping

import time
import scipy.fftpack
import operator

class plotter():

    def robotx(self,data):

        self.t = []
        self.a = []
        self.b = []
        self.c = []
        self.d = []

        channels = data.channels
        #samples = float(data.samples/channels)
        #print samples
        sample_rate = data.sample_rate
        #print sample_rate
        #adc_bit = data.adc_bit
        values = data.data

        values = [((x - 32768.0)/65536.0)*5.0 for x in values ]

        #values = values[15000:21000:1]
        
        samples = float(data.samples) #float(len(data)/channels)

        time = (samples/float(sample_rate))#*10**6

        #print("data received from service")

        length = samples*channels
        self.x_axis_length = (samples/4.0)*(1.0/sample_rate)

        #print samples
        #print self.x_axis_length

        starting_sample = 0
        distance = (float(length)/float(samples))*time

        #self.x = np.arange(starting_sample*(time/samples),distance, time/samples)
        #print self.x
        self.a = values[starting_sample + 0:int(length):channels]
        #print self.a
        self.b = values[starting_sample + 1:int(length):channels]
        self.c = values[starting_sample + 2:int(length):channels]
        self.d = values[starting_sample + 3:int(length):channels]

        #print len(self.a1)
        #print range(len(self.a1)-1)

        self.a1 = [0]*int(samples/4)
        self.b1 = [0]*int(samples/4)
        self.c1 = [0]*int(samples/4)
        self.d1 = [0]*int(samples/4)
        #print samples/4

        '''for i in range(len(self.a)-3):
            #print i
            self.a1[i] = self.a[i]*(0.25) + self.a[i+1]*(0.25) + self.a[i+2]*(0.25) + self.a[i+3]*(0.25)
            self.b1[i] = self.b[i]*(0.25) + self.b[i+1]*(0.25) + self.b[i+2]*(0.25) + self.b[i+3]*(0.25)
            self.c1[i] = self.c[i]*(0.25) + self.c[i+1]*(0.25) + self.c[i+2]*(0.25) + self.c[i+3]*(0.25)
            self.d1[i] = self.d[i]*(0.25) + self.d[i+1]*(0.25) + self.d[i+2]*(0.25) + self.d[i+3]*(0.25)'''

        self.N = len(self.a)   #number of samples in channel1
        Fs = sample_rate
        Ts = 1.0/Fs    
        #x = np.linspace(0.0, N*Ts, N)    
        self.yf = scipy.fftpack.fft(self.a)
        index, value = max(enumerate(abs(self.yf)), key=operator.itemgetter(1))

        self.xf = np.linspace(0.0, 1.0/(2.0*Ts), self.N/2)

        legends = [None]*channels  #set up ledgends for x channels
        wave = [None]*len(self.a)  #make empty list

        n = len(self.a)   #length of samples (samplecount/4 from AdvanTech driver c++)

        self.t = np.arange(0,n*Ts,Ts)  #resolution of sampling, ie 1 MS/s = 1*10^-6

        if len(self.t) > n:
            self.t = self.t[:-1]  #make sure len(t) is = to len(n), shave the last number off



    def determine_phase(self, ref_sig, a_sig):
        channel_length = len(ref_sig)

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

        #print len(reference)

        cross_corr = np.correlate(reference, signal, mode='full')
        max_idx = cross_corr.argmax()
        #print "prior: %0.2f best: %0.2f after: %0.2f" % (cross_corr[max_idx-1], cross_corr[max_idx], cross_corr[max_idx+1])

        #rospy.loginfo(len(cross_corr))
        #print len(reference)
        #plt.plot(cross_corr)
        #plt.ylabel('some numbers')
        #plt.show()


        
        phase_holder = max_idx - (len(reference) + 0)
        #print phase_holder

        
        
        return cross_corr 


    def plot_ping(self,data):       

        self.t = []
        self.a = []
        self.b = []
        self.c = []
        self.d = []

        channels = data.channels
        #samples = float(data.samples/channels)
        #print samples
        sample_rate = data.sample_rate
        #print sample_rate
        adc_bit = data.adc_bit
        values = data.data

        #values = values[15000:21000:1]
        
        samples = float(data.samples) #float(len(data)/channels)
        #16,000 samples or 4,000 samples per channel
        #rospy.logwarn(samples)

        time = ((samples/4.0)/float(sample_rate))#*10**6

        #print("data received from service")

        length = samples*channels
        self.x_axis_length = (samples/4.0)*(1.0/sample_rate)

        #print sample_rate
        #print samples
        #print self.x_axis_length

        starting_sample = 0
        distance = (float(length)/float(samples))*time

        #self.x = np.arange(starting_sample*(time/samples),distance, time/samples)
        #print self.x
        self.a = values[starting_sample + 0:int(length):channels]
        #print self.a
        self.b = values[starting_sample + 1:int(length):channels]
        self.c = values[starting_sample + 2:int(length):channels]
        self.d = values[starting_sample + 3:int(length):channels]

        self.bc = self.determine_phase(self.a, self.b)
        self.cc = self.determine_phase(self.a, self.c)
        self.dc = self.determine_phase(self.a, self.d)

        #print len(self.b)

        #print len(self.a1)
        #print range(len(self.a1)-1)

        self.a1 = [0]*int(samples/4)
        self.b1 = [0]*int(samples/4)
        self.c1 = [0]*int(samples/4)
        self.d1 = [0]*int(samples/4)
        #print samples/4

        '''for i in range(len(self.a)-3):
            #print i
            self.a1[i] = self.a[i]*(0.25) + self.a[i+1]*(0.25) + self.a[i+2]*(0.25) + self.a[i+3]*(0.25)
            self.b1[i] = self.b[i]*(0.25) + self.b[i+1]*(0.25) + self.b[i+2]*(0.25) + self.b[i+3]*(0.25)
            self.c1[i] = self.c[i]*(0.25) + self.c[i+1]*(0.25) + self.c[i+2]*(0.25) + self.c[i+3]*(0.25)
            self.d1[i] = self.d[i]*(0.25) + self.d[i+1]*(0.25) + self.d[i+2]*(0.25) + self.d[i+3]*(0.25)'''

        self.N = len(self.a)   #number of samples in channel1
        Fs = sample_rate
        Ts = 1.0/Fs    
        #x = np.linspace(0.0, N*Ts, N)    
        self.yf = scipy.fftpack.fft(self.a)
        index, value = max(enumerate(abs(self.yf)), key=operator.itemgetter(1))

        self.xf = np.linspace(0.0, 1.0/(2.0*Ts), self.N/2)

        legends = [None]*channels  #set up ledgends for x channels
        wave = [None]*len(self.a)  #make empty list
        

        n = len(self.a)   #length of samples (samplecount/4 from AdvanTech driver c++)
        

        self.t = np.arange(0,n*Ts,Ts)  #resolution of sampling, ie 1 MS/s = 1*10^-6

        if len(self.t) > n:
            self.t = self.t[:-1]  #make sure len(t) is = to len(n), shave the last number off     

        

        #print self.t       


    def __init__(self):
        rospy.init_node('ping_plotter')
        #rospy.Subscriber('/hydrophones/ping', Ping, self.signal_plotter)

        rate = rospy.Rate(10)

        #self.x = []
        self.a = []
        self.a1 = []
        self.b = []
        self.c = []
        self.d = []
        self.trigger = False
        self.t  = [0]*2873
        self.x_axis_length = 0
        self.xf = 0
        self.N = 1
        self.yf = [0]

        self.bc  = [0]*2873
        self.cc  = [0]*2873
        self.dc  = [0]*2873

        plt.ion()
        plt.plot()         
        #fig, self.ax = plt.subplots(1, 1)  

        while not rospy.is_shutdown():
            #ping_service = rospy.ServiceProxy('hydrophones/ping', Ping)
            #ping = ping_service()

            #rospy.Subscriber('/hydrophones/pingmsg', Pingdata, self.plot_ping) #for simulation
            #rospy.Subscriber('/hydrophones/pingraw', Pingdata, self.plot_ping)
            rospy.Subscriber('/hydrophones/pingconditioned', Pingdata, self.plot_ping)
            #rospy.Subscriber('hydrophones/ping', Ping, self.robotx)

            if len(self.t) == len(self.a):
                xvalues = []
                avalues = []
                bvalues = []
                cvalues = []
                dvalues = []

                self.t = self.t
                xvalues = self.t
                avalues = self.a
                bvalues = self.b
                cvalues = self.c
                dvalues = self.d
                Nval = self.N
                xfval = self.xf
                yfval = self.yf

                #print len(xvalues)
                #print len(bvalues)

                plt.cla()
                #print "%i, %i" % (len(self.t), len(self.a))
                line_a = plt.plot(xvalues,avalues, linewidth=3.0, label='Hydrophone A')
                line_b = plt.plot(xvalues,bvalues, linewidth=3.0, label='Hydrophone B')
                line_c = plt.plot(xvalues,cvalues, linewidth=3.0, label='Hydrophone C')
                line_d = plt.plot(xvalues,dvalues, linewidth=3.0, label='Hydrophone D')

                plt.legend(loc="upper left")#, fontsize=25)
                plt.title("Conditioned Signals", weight = 'bold', size = 37, x = 0.5, y = 1.02, horizontalalignment='center')
                plt.xlabel('Time (sec)', size = 25, weight = 'bold', x = 0.5, y = 0)
                plt.ylabel('Amplitude (V)', size = 25, weight = 'bold', x = 0, y = 0.5)
                #self.ax[0].set_ylim(-5,5)
                plt.xlim(0,self.x_axis_length)
                plt.tick_params(axis='both', which='major', labelsize=25, pad=20)
                plt.tick_params(axis='both', which='minor', labelsize=25, pad=20)
                #plt.xaxis.labelpad = 20
                #plt.yaxis.labelpad = 20


                '''self.ax[1].cla()
                #self.ax[2].set_title("FFT On Channel One")
                #self.ax[1].plot(frq,abs(Y),'r') # plotting the FFT spectrum
                if self.yf[0] != 0:
                    self.ax[1].plot(xfval,2.0/Nval * np.abs(yfval[:Nval//2]),'r') # plotting the FFT spectrum
                #print abs(Y)
                self.ax[1].set_xlim(5000,50000)
                #plt.xticks(np.arange(5000, 50000+1, 500.0))
                #self.ax[1].set_ylim(0,n/10)
                self.ax[1].set_xlabel('Freq (Hz)')
                self.ax[1].set_ylabel('|Y(freq)|')'''

                plt.pause(0.05)
                self.plot = 0

            rate.sleep()

        plt.close('all')

def main():
    rospy.init_node('ping_plotter', anonymous=False)

    plotter()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print "Shutting down"
        pass
    


if __name__ == '__main__':
    main()