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

import time
import scipy.fftpack
import operator

class plotter():

    def __init__(self):
        rospy.init_node('ping_plotter')
        #rospy.Subscriber('/hydrophones/ping', Ping, self.signal_plotter)

        rate = rospy.Rate(10)

        #self.x = []
        self.a = []
        self.b = []
        self.c = []
        self.d = []
        self.trigger = False

        plt.ion()
        fig, ax = plt.subplots(2, 1)

        while not rospy.is_shutdown():
            ping_service = rospy.ServiceProxy('hydrophones/ping', Ping)
            ping = ping_service()

            channels = ping.channels
            samples = float(ping.samples/channels)
            #print samples
            sample_rate = ping.sample_rate
            #print sample_rate
            adc_bit = ping.adc_bit
            data = ping.data

            time = (samples/float(sample_rate))#*10**6

            #print("data received from service")

            length = samples*channels
            x_axis_length = samples*(1.0/sample_rate)
            starting_sample = 0
            distance = (float(length)/float(samples))*time

            #self.x = np.arange(starting_sample*(time/samples),distance, time/samples)
            #print self.x
            self.a = data[starting_sample + 0:int(length):2]
            #print self.a
            self.b = data[starting_sample + 1:int(length):2]

            signal = 'bad' 

            N = len(self.a)   #number of samples in channel1
            Fs = sample_rate
            Ts = 1.0/Fs    
            #x = np.linspace(0.0, N*Ts, N)    
            yf = scipy.fftpack.fft(self.a)
            index, value = max(enumerate(abs(yf)), key=operator.itemgetter(1))
            print "**************************"
            print "**************************"
            print "**************************"
            print "**************************"
            print "length of whole list: ", len(yf)
            print "index in whole list: ", index
            print "max value in whole list: ", abs(value)
            
            front_half = self.a[:len(self.a)/2]
            yf_1 = scipy.fftpack.fft(front_half)
            indexf, value = max(enumerate(abs(yf_1)), key=operator.itemgetter(1))
            print "**************************"
            print "length of front_half: ", len(yf_1)
            print "index in front_half: ", index
            print "max value in front_half: ", abs(value)            
            
            back_half  = self.a[len(self.a)/2:]
            yf_2 = scipy.fftpack.fft(back_half )
            indexb, value = max(enumerate(abs(yf_2)), key=operator.itemgetter(1))
            print "**************************"
            print "length of back_half: ", len(yf_2)
            print "index in back_half: ", index
            print "max value in back_half: ", abs(value)

            if indexf != indexb and index >= 25:
                signal = 'good'
                print "GOOD SIGNAL GOOD SIGNAL GOOD SIGNAL!!!!!!!"

            if signal == "good":
                xf = np.linspace(0.0, 1.0/(2.0*Ts), N/2)


                legends = [None]*channels  #set up ledgends for x channels
                wave = [None]*len(self.a)  #make empty list

                n = len(self.a)   #length of samples (samplecount/4 from AdvanTech driver c++)

                t = np.arange(0,n*Ts,Ts)  #resolution of sampling, ie 1 MS/s = 1*10^-6

                if len(t) > n:
                    #print t
                    t = t[:-1]  #make sure len(t) is = to len(n), shave the last number off            

                ax[0].cla()
                ax[0].plot(t,self.a, linewidth=2.0, label='Hydrophone 0')
                ax[0].plot(t,self.b, linewidth=2.0, label='Hydrophone 1')

                ax[0].legend(loc="upper left", fontsize=25)
                ax[0].set_title("Actual Received Signals", weight = 'bold', size = 37, x = 0.5, y = 1.02, horizontalalignment='center')
                ax[0].set_xlabel('Time (seconds)', size = 25, weight = 'bold', x = 0.5, y = 0)
                ax[0].set_ylabel('Amplitude', size = 25, weight = 'bold', x = 0, y = 0.5)
                ax[0].set_ylim(-0.5,0.5)
                ax[0].set_xlim(0,x_axis_length)
                ax[0].tick_params(axis='both', which='major', labelsize=25, pad=20)
                ax[0].tick_params(axis='both', which='minor', labelsize=25, pad=20)
                ax[0].xaxis.labelpad = 20
                ax[0].yaxis.labelpad = 20


                ax[1].cla()
                #ax[2].set_title("FFT On Channel One")
                #ax[1].plot(frq,abs(Y),'r') # plotting the FFT spectrum
                ax[1].plot(xf,2.0/N * np.abs(yf[:N//2]),'r') # plotting the FFT spectrum
                #print abs(Y)
                ax[1].set_xlim(5000,50000)
                #plt.xticks(np.arange(5000, 50000+1, 500.0))
                #ax[1].set_ylim(0,n/10)
                ax[1].set_xlabel('Freq (Hz)')
                ax[1].set_ylabel('|Y(freq)|')



                plt.pause(0.05)

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