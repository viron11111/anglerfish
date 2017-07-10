#!/usr/bin/env python
import rospy
import rosparam
import random

import numpy as np
from scipy import optimize

from sub8_msgs.srv import Sonar, SonarResponse

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from std_msgs.msg import UInt16
from pinger_tracker.msg import *

import time
from multilateration import Multilaterator, ReceiverArraySim, Pulse

class simulator():

    def create_time_stamps(self, position):           

        hydrophone_locations = {   
        'hydro0': {'x':       0, 'y':       0, 'z':      0},
        'hydro1': {'x':   -25.4, 'y':       0, 'z':      0},
        'hydro2': {'x':    25.4, 'y':       0, 'z':      0},
        'hydro3': {'x':       0, 'y':   -25.4, 'z':      0}}

        c = 1.484  # millimeters/microsecond
        hydrophone_array = ReceiverArraySim(hydrophone_locations, c)
        sonar = Multilaterator(hydrophone_locations, c, 'LS')

        pulse = Pulse(position[0], position[1], position[2], 0)
        #print pulse
        tstamps = hydrophone_array.listen(pulse)
        tstamps = tstamps - tstamps[0]
        return tstamps

    def create_wave(self, offset):
        self.Fs = self.sample_rate  # sampling rate
        self.Ts = 1.0/self.Fs # sampling interval

        self.samples = (0.0004-offset)*self.sample_rate  #Number of samples during a 400 uSec period, for pre_signal
        #print self.samples

        pre_signal = [(2**self.resolution)/2.0]*int(self.samples)  #dead period prior to signal


        t = np.arange(0,0.0004+offset,self.Ts) # time vector for signal waves
        print len(t)-int(self.samples)

        ff = self.signal_freq   # frequency of the signal
        y = np.sin(2*np.pi*ff*t+self.phase_jitter)*(self.amplitude*self.amplitude_jitter) + 1

        y = (y/2) * 2**self.resolution

        #print y

        y = np.array(y,dtype=int)
     

        #pre_signal = np.array(pre_signal,dtype=int)

        wave = np.append(pre_signal,y)  #append silence before signal to actual signal

        return wave



    def __init__(self):
        rospy.init_node('signal_simulator')

        self.sample_rate = rospy.get_param('~sample_rate', 300e3)
        #self.thresh = rospy.get_param('~thresh', 500)
        self.frame = rospy.get_param('~frame', '/hydrophones')
        #permute_str = rospy.get_param('~permute', '1 2 3 4')
        #self.samples = rospy.get_param('sample_number', 1024)
        self.resolution = rospy.get_param('resolution', 16)
        self.signal_freq = rospy.get_param('signal_freq', 43e3)
        self.amplitude = rospy.get_param('amplitude', 1.0)
        self.number_of_hydrophones = rospy.get_param('number_of_hydrophones', 4)

        self.signal_pub = rospy.Publisher('/hydrophones/ping', Ping, queue_size = 1)


        self.Fs = self.sample_rate  # sampling rate
        self.Ts = 1.0/self.Fs # sampling interval

        
        #print jitter

        shift = random.uniform(-0.00001,0.00001)

        position = (10000, 1000, -1000)
        shift = self.create_time_stamps(position)
        tstamps = shift*10**-6

        

        plt.ion()
        fig, ax = plt.subplots(2, 1)

        rate = rospy.Rate(2)

        while not rospy.is_shutdown():

            phase_jitter = ((1/self.sample_rate)/(1/self.signal_freq))*np.pi
            self.phase_jitter = random.uniform(-phase_jitter/2,phase_jitter/2)         
            

            ax[0].cla()
            ax[0].set_title("Four Hydrophone Channels Full Scale")

            for i in range(0,4):
                
                self.amplitude_jitter = random.uniform(0.8,1.0)
                for i in range(0,4):
                    #print tstamps[i]
                    wave = self.create_wave(tstamps[i])

                n = len(wave)
                t = np.arange(0,n*self.Ts,self.Ts)

                k = np.arange(n)
                T = float(n)/float(self.Fs)            
                frq = k/T # two sides frequency range

                frq = frq[range(n/2)] # one side frequency range
                Y = np.fft.fft(wave)/n # fft computing and normalization
                Y = Y[range(n/2)]/2**self.resolution
                
                ax[0].plot(t,wave)


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

'''        def signal_plotter(self, data):
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
        print time
        print distance
        print time/samples
        print time/(time/samples)

        self.x = np.arange(starting_sample*(time/samples),distance, time/samples)
        #print self.x
        self.a = values[starting_sample*4 + 0:length*4:4]
        #print self.a
        self.b = values[starting_sample*4 + 1:length*4:4]
        self.c = values[starting_sample*4 + 2:length*4:4]
        self.d = values[starting_sample*4 + 3:length*4:4]

        self.trigger = True
        #s = 1 + np.sin(2*np.pi*t)
'''