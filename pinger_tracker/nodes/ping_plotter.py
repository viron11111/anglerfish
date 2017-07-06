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

        while not rospy.is_shutdown():
            #print self.trigger
            if self.trigger == True:
                #freq = np.fft.fft(self.x, self.a)
                #freq = np.abs(freq)
                #print freq

                Fs = 300000
                Ts = 1.0/Fs
                

                y = self.a
                n = len(y)

                t = np.arange(0,n*Ts,Ts)

                k = np.arange(n)
                T = n/Fs
                frq = k/T # two sides frequency range
                frq = frq[range(n/2)] # one side frequency range

                Y = np.fft.fft(y)/n # fft computing and normalization
                Y = Y[range(n/2)]

                fig, ax = plt.subplots(2, 1)
                ax[0].plot(t,y)
                ax[0].set_xlabel('Time')
                ax[0].set_ylabel('Amplitude')
                ax[1].plot(frq,abs(Y),'r') # plotting the spectrum
                ax[1].set_xlabel('Freq (Hz)')
                ax[1].set_ylabel('|Y(freq)|')

                #plot_url = py.plot_mpl(fig, filename='mpl-basic-fft')




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