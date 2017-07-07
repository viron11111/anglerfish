#!/usr/bin/env python
import rospy
import rosparam

import numpy as np
from scipy import optimize

from sub8_msgs.srv import Sonar, SonarResponse

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from std_msgs.msg import UInt16
from pinger_tracker.msg import *

import time

class simulator():

    def signal_plotter(self, data):
        #print "msg received"
        values = data.data
        samples = float(data.samples/4)
        sample_rate = float(data.sample_rate)
        time = (samples/sample_rate)*10**6
        #print time

        #print "msg"

        length = 256
        starting_sample = 0
        distance = (length/samples)*time
        '''print time
        print distance
        print time/samples
        print time/(time/samples)'''

        self.x = np.arange(starting_sample*(time/samples),distance, time/samples)
        #print self.x
        self.a = values[starting_sample*4 + 0:length*4:4]
        #print self.a
        self.b = values[starting_sample*4 + 1:length*4:4]
        self.c = values[starting_sample*4 + 2:length*4:4]
        self.d = values[starting_sample*4 + 3:length*4:4]

        self.trigger = True
        #s = 1 + np.sin(2*np.pi*t)


    def __init__(self):
        rospy.init_node('signal_simulator')

        self.sample_rate = rospy.get_param('~sample_rate', 300e3)
        #self.thresh = rospy.get_param('~thresh', 500)
        self.frame = rospy.get_param('~frame', '/hydrophones')
        #permute_str = rospy.get_param('~permute', '1 2 3 4')
        #self.samples = rospy.get_param('sample_number', 1024)
        self.resolution = rospy.get_param('resolution', 16)
        self.signal_freq = rospy.get_param('signal_freq', 43e3)
        self.amplitude = rospy.get_param('amplitude', 0.08)

        self.signal_pub = rospy.Publisher('/hydrophones/ping', Ping, queue_size = 1)

        plt.ion()
        fig, ax = plt.subplots(2, 1)

        self.samples = 0.0004*self.sample_rate

        pre_signal = [(2**self.resolution)/2.0]*int(self.samples)

        print self.samples

        Fs = self.sample_rate  # sampling rate
        Ts = 1.0/Fs # sampling interval
        t = np.arange(0,0.0004,Ts) # time vector

        ff = self.signal_freq   # frequency of the signal
        y = np.sin(2*np.pi*ff*t)*self.amplitude + 1

        y = (y/2) * 2**self.resolution

        #print y

        y = np.array(y,dtype=int)
        

        #pre_signal = np.array(pre_signal,dtype=int)

        y = np.append(pre_signal,y)

        n = len(y)
        t = np.arange(0,n*Ts,Ts)

        k = np.arange(n)
        T = float(n)/float(Fs)            
        frq = k/T # two sides frequency range

        frq = frq[range(n/2)] # one side frequency range
        Y = np.fft.fft(y)/n # fft computing and normalization
        Y = Y[range(n/2)]/2**self.resolution





        '''for i in range(len(y-1)):
            y[i] = int(y[i])
            #print ("%i" % y[i])

        print y'''

        rate = rospy.Rate(2)

        while not rospy.is_shutdown():
            
            ax[0].cla()
            ax[0].set_title("Four Hydrophone Channels Full Scale")
            ax[0].plot(t,y)
            ax[0].set_ylim(0,2**self.resolution)
            ax[0].set_xlabel('Time')
            ax[0].set_ylabel('Amplitude')

            ax[1].axvline(30000)
            ax[1].cla()
            ax[1].set_title("FFT On Channel One")
            ax[1].plot(frq,abs(Y),'r') # plotting the spectrum
            ax[1].set_xlim(5000,50000)
            #ax[1].set_ylim(0,500)
            ax[1].set_xlabel('Freq (Hz)')
            ax[1].set_ylabel('|Y(freq)|')

            plt.pause(0.05)
            
            rate.sleep()

        plt.close('all')

def main():
    rospy.init_node('signal_simulator', anonymous=False)

    simulator()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print "Shutting down"
        pass
    


if __name__ == '__main__':
    main()