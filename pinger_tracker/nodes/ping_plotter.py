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

class plotter():

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
        rospy.init_node('ping_plotter')
        rospy.Subscriber('/hydrophones/ping', Ping, self.signal_plotter)

        rate = rospy.Rate(5)

        self.x = []
        self.a = []
        self.b = []
        self.c = []
        self.d = []
        self.trigger = False

        plt.ion()
        fig, ax = plt.subplots(3, 1)

        while not rospy.is_shutdown():
            #print self.trigger
            if self.trigger == True:
                #freq = np.fft.fft(self.x, self.a)
                #freq = np.abs(freq)
                #print freq                

                Fs = 300000
                Ts = 1.0/Fs        

                wave = [None]*len(self.a)

                if len(self.a) != 0:
                    for i in range(len(self.a)):
                        wave[i] = self.a[i]        

                y = self.a
                #print wave
                n = len(y)

                t = np.arange(0,n*Ts,Ts)

                y = wave[n/2:n]
                n = len(y)

                k = np.arange(n)
                T = float(n)/float(Fs)            
                frq = k/T # two sides frequency range

                frq = frq[range(n/2)] # one side frequency range
                Y = np.fft.fft(y)/n # fft computing and normalization
                Y = Y[range(n/2)]/2**16

                '''for i in range(5,n/2):
                    
                    if abs(Y[i]) > 80:
                        print "ping detected"
                        left_line = np.arange(0,300,300)'''

                ax[0].cla()
                ax[0].set_title("Four Hydrophone Channels Full Scale")
                ax[0].plot(t,self.a)
                ax[0].plot(t,self.b)
                ax[0].plot(t,self.c)
                ax[0].plot(t,self.d)
                ax[0].set_ylim(0,65536)
                ax[0].set_xlabel('Time')
                ax[0].set_ylabel('Amplitude')

                ax[1].cla()
                ax[1].set_title("Four Hydrophone Channels Autosized")
                ax[1].plot(t,self.a)
                ax[1].plot(t,self.b)
                ax[1].plot(t,self.c)
                ax[1].plot(t,self.d)
                ax[1].set_xlabel('Time')
                ax[1].set_ylabel('Amplitude')

                #ax[2].axvline(30000)
                ax[2].cla()
                ax[2].set_title("FFT On Channel One")
                ax[2].plot(frq,abs(Y),'r') # plotting the spectrum
                ax[2].set_xlim(5000,50000)
                #ax[2].set_ylim(0,300)
                ax[2].set_xlabel('Freq (Hz)')
                ax[2].set_ylabel('|Y(freq)|')

                #plot_url = py.plot_mpl(fig, filename='mpl-basic-fft')
                plt.pause(0.05)
                self.trigger = False

                #802.3af 48V

                '''plt.gcf().clear()
                plt.plot(self.x, self.a)
                #plt.plot(self.x, self.b)
                #plt.plot(self.x, self.c)
                #plt.plot(self.x, self.d)

                plt.xlabel('time (uS)')
                plt.ylabel('ADC')
                #plt.ylim(30500,35000)
                plt.title('X')
                plt.grid(True)
                plt.pause(0.05)
                self.trigger = False'''

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