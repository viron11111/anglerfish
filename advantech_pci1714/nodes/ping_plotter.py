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
        fig, ax = plt.subplots(1, 1)

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
 

            Fs = sample_rate
            Ts = 1.0/Fs        

            legends = [None]*channels
            wave = [None]*len(self.a)

            if len(self.a) != 0:
                for i in range(len(self.a)):
                    wave[i] = self.a[i]        

            y = self.a

            n = len(y)

            #print len(self.a)
            #print n

            t = np.arange(0,n*Ts,Ts)

            if len(t) > n:
                #print t
                t = t[:-1]

            y = wave[n/2:n]
            n = len(y)

            k = np.arange(n)
            T = float(n)/float(Fs)            
            frq = k/T # two sides frequency range

            frq = frq[range(n/2)] # one side frequency range
            Y = np.fft.fft(y)/n # fft computing and normalization
            Y = Y[range(n/2)]/2**16

            #print t

            ax.cla()
            #ax.set_title("Four Hydrophone Channels Autosized")
            ax.plot(t,self.a, linewidth=2.0, label='Hydrophone 0')
            ax.plot(t,self.b, linewidth=2.0, label='Hydrophone 1')
            #ax.plot(t,self.c, linewidth=2.0, label='Hydrophone 2')
            #ax.plot(t,self.d, linewidth=2.0, label='Hydrophone 3')
            #ax[1].set_xlabel('Time')

            ax.legend(loc="upper left", fontsize=25)
            ax.set_title("Actual Received Signals", weight = 'bold', size = 37, x = 0.5, y = 1.02, horizontalalignment='center')
            ax.set_xlabel('Time (seconds)', size = 25, weight = 'bold', x = 0.5, y = 0)
            ax.set_ylabel('Amplitude', size = 25, weight = 'bold', x = 0, y = 0.5)
            ax.set_xlim(0,x_axis_length)
            ax.tick_params(axis='both', which='major', labelsize=25, pad=20)
            ax.tick_params(axis='both', which='minor', labelsize=25, pad=20)
            ax.xaxis.labelpad = 20
            ax.yaxis.labelpad = 20

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