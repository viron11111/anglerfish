#!/usr/bin/env python
import rospy
import rosparam
import random
import numpy as np
import matplotlib.pyplot as plt

from std_msgs.msg import Header, Bool
from pinger_tracker.msg import *
from multilateration import Multilaterator, ReceiverArraySim, Pulse

import sys

from dynamic_reconfigure.server import Server
from pinger_tracker.cfg import SignalConfig

import signal

import os



#*********************************************************************
#  Using mix of uSec and sec, pay attention to units
#  Hydrophone locations are in mm, along with speed of sound

class simulator():
    def hydrophone_locations(self, data):
        self.hydro0 = [data.hydro0_xyz[0],data.hydro0_xyz[1],data.hydro0_xyz[2]]
        self.hydro1 = [data.hydro1_xyz[0],data.hydro1_xyz[1],data.hydro1_xyz[2]]
        self.hydro2 = [data.hydro2_xyz[0],data.hydro2_xyz[1],data.hydro2_xyz[2]]
        self.hydro3 = [data.hydro3_xyz[0],data.hydro3_xyz[1],data.hydro3_xyz[2]]

    def callback_signal(self, config, level):
        self.resolution = int("{ADC_bits}".format(**config))
        self.sample_rate = int("{sample_rate}".format(**config))
        self.tx_rate = float("{signal_rate}".format(**config))
        self.amplitude = float("{amplitude}".format(**config))
        self.signal_freq = int("{signal_freq}".format(**config))
        self.signal_trigger = "{signal_gen_trigger}".format(**config)

        self.position[0] = float("{pinger_x_pos}".format(**config))
        self.position[1] = float("{pinger_y_pos}".format(**config))
        self.position[2] = float("{pinger_z_pos}".format(**config))

        self.noise_sync = "{synced_signal_noise}".format(**config)
        self.signal_noise = float("{signal_noise}".format(**config))
        
        return config

    def create_time_stamps(self, position):           

        hydrophone_locations = {   
        'hydro0': {'x':  self.hydro0[0], 'y':   self.hydro0[1], 'z':  self.hydro0[2]},
        'hydro1': {'x':  self.hydro1[0], 'y':   self.hydro1[1], 'z':  self.hydro1[2]},
        'hydro2': {'x':  self.hydro2[0], 'y':   self.hydro2[1], 'z':  self.hydro2[2]},
        'hydro3': {'x':  self.hydro3[0], 'y':   self.hydro3[1], 'z':  self.hydro3[2]}}

        c = 1.484  # millimeters/microsecond
        hydrophone_array = ReceiverArraySim(hydrophone_locations, c)
        #sonar = Multilaterator(hydrophone_locations, c, 'LS')

        pulse = Pulse(position[0], position[1], position[2], 0)
        tstamps = hydrophone_array.listen(pulse)
        tstamps = tstamps - tstamps[0]
        return tstamps

    def create_silence(self, offset):

        self.samples = ((self.signal_length/2)-offset)*self.sample_rate*1000  #Number of samples during half the signal, for pre_signal


        pre_signal = [(2**self.resolution)/2.0]*int(self.samples)  #dead period prior to signal

        if len(pre_signal) == len(self.noise[0:len(pre_signal)]):
            pre_signal = pre_signal + self.noise[0:len(pre_signal)]*random.uniform(1.0,2) #add noise with noise multiplier

        return pre_signal


    def create_wave(self, offset):

        self.Fs = self.sample_rate*1000  # sampling rate
        self.Ts = 1.0/self.Fs # sampling interval

        self.samples = ((self.signal_length/2)-offset)*self.sample_rate*1000  #Number of samples during a 400 uSec period, for pre_signal
        int_sample = self.samples
        int_sample = int(int_sample)

        pre_signal = self.create_silence(offset) #[(2**self.resolution)/2.0]*int(self.samples)  #dead period prior to signal


        t = np.arange(0.0,(self.signal_length/2)+offset,self.Ts) # time vector for signal waves

        total = len(t)+int_sample

        if total > self.signal_length/self.Ts:

            t = t[:len(t)-1]

        ff = self.signal_freq*1000.0   # frequency of the signal, 43 kHz for Anglerfish

        #self.phase_jitter randomly places phase within on sampling time
        #amplitude jitter for realism
        y = np.sin(2*np.pi*ff*t+self.phase_jitter)*(self.amplitude*self.amplitude_jitter) + 1  #create sine wave

        y = (y/2) * 2**self.resolution  #turn sine wave into an int

        y = np.array(y,dtype=int) #Help from Kevin, turn Floats in y to Int

        if self.noise_sync == 'True':
            x = self.noise*int(100*self.signal_noise)

            y = [q + r for q, r in zip(y, x)]

        elif self.noise_sync == 'False':
            self.noise = np.random.normal(-((2**self.resolution)*0.0005)/2,((2**self.resolution)*0.0005)/2,(int(self.signal_length/self.Ts)))

            #turn Float noise into Int noise
            for i in range(0,len(self.noise)):
                self.noise[i] = int(self.noise[i])
            
            x = self.noise*int(100*self.signal_noise)

            y = [q + r for q, r in zip(y, x)]                


        wave_func = np.append(pre_signal,y)  #append silence before signal to actual signal

        return wave_func

    def get_pos(self, data):
        self.position = (data.x_pos, data.y_pos, data.z_pos)

    def trigger_func(self,data):
        self.trigger = data.data      
    
    def signal_handler(self, signal, frame):
        os.system("rosnode kill signal_simulator")   

    def __init__(self):
        signal.signal(signal.SIGINT, self.signal_handler)
        rospy.init_node('signal_simulator')
        self.position = [3000, 5000, -2000]  # in mm, default position 

        srv = Server(SignalConfig, self.callback_signal)

        rospy.Subscriber('hydrophones/simulated_position', Transmitter_position, self.get_pos)
        rospy.Subscriber('hydrophones/signal_trigger', Bool, self.trigger_func)

        self.trigger = 'False'

        self.simulate_pub = rospy.Publisher('hydrophones/ping', Ping, queue_size = 1)

        self.hydro0 = [0,     0,     0]
        self.hydro1 = [-25.4, 0,     0]
        self.hydro2 = [25.4,  0,     0]
        self.hydro3 = [0,     -25.4, 0]

        rospy.Subscriber('hydrophones/hydrophone_locations', Hydrophone_locations, self.hydrophone_locations)

        self.sample_rate = rospy.get_param('~sample_rate', 600)  #ADC sampling rate
        #self.thresh = rospy.get_param('~thresh', 500)
        self.frame = rospy.get_param('~frame', '/hydrophones')
        #permute_str = rospy.get_param('~permute', '1 2 3 4')
        #self.samples = rospy.get_param('sample_number', 1024)
        self.resolution = rospy.get_param('resolution', 16)  #ADC bits
        self.signal_freq = rospy.get_param('signal_freq', 27)  #pinger freq
        self.amplitude = rospy.get_param('amplitude', 0.1)      #received signal amplitude 0.0-1.0
        self.number_of_hydrophones = rospy.get_param('number_of_hydrophones', 4)  
        self.signal_length = rospy.get_param('signal_length', 0.0008)  #800 uSec from default paul board

        self.signal_pub = rospy.Publisher('/hydrophones/ping', Ping, queue_size = 1)
        self.tstamps_pub = rospy.Publisher('/hydrophones/actual_time_stamps', Actual_time_stamps, queue_size = 1)

        self.tx_rate = 1.0

        self.Fs = self.sample_rate*1000  # sampling rate
        self.Ts = 1.0/self.Fs # sampling interval

        plt.ion()
        fig, ax = plt.subplots(3, 1)  #3x1 plot 

        rate = rospy.Rate(1)  #rate of signals, 5 Hz for Anglerfish
        catch = 0  
        self.trigger = 0
        self.signal_trigger = 'False'
        interrupted = False

        while not rospy.is_shutdown():

            #if self.trigger == 0 and catch == 1:
            #    catch = 0

            if self.signal_trigger == 'False':
                catch = 0
                rate = rospy.Rate(self.tx_rate)  #rate of signals, 5 Hz for Anglerfish
            
            elif self.signal_trigger == 'True' and catch == 0:
                while(self.trigger == 0 and self.signal_trigger == 'True'):
                    if interrupted:
                        break                        
                catch = 1
            elif self.trigger == 1 and self.signal_trigger =='True':
                while(self.trigger == 1 and self.signal_trigger == 'True'):
                    if interrupted:
                        break                   
                catch = 0           

            tstamps = self.create_time_stamps(self.position)

            #converts timestamps to Sec because create_time_stamps uses uSec
            for i in range(0,4):
                tstamps[i] = tstamps[i]*10**-6

            self.tstamps=tstamps


            microseconds = [1e6,1e6,1e6,1e6]
            self.tstamps = [x * y for x, y in zip(self.tstamps,microseconds)]
            self.tstamps = list(self.tstamps)

            self.tstamps_pub.publish(Actual_time_stamps(
                    header=Header(stamp=rospy.Time.now(),
                                  frame_id='signal_sim'),
                    actual_time_stamps=self.tstamps))

            #phase jitter, shifts sine wave left or right within one sampling period (1/300000 sec for Paul board)
            phase_jitter = ((1.0/float(self.sample_rate*1000))/(1.0/(self.signal_freq*1000)))*np.pi
            self.phase_jitter = random.uniform(-phase_jitter/2,phase_jitter/2)    
            
            ax[0].cla()
            ax[1].cla()

            #self.noise is used to add noise to the silent portion of the signal            
            self.noise = np.random.normal(-((2**self.resolution)*0.0005)/2,((2**self.resolution)*0.0005)/2,(int(self.signal_length/self.Ts)))

            #turn Float noise into Int noise
            for i in range(0,len(self.noise)):
                self.noise[i] = int(self.noise[i])
            
            #count the number of published data point for assignment of empty self.data list
            self.data_points = int(self.signal_length/self.Ts)*self.number_of_hydrophones
            self.data = [None]*self.data_points

            for i in range(0,4):  #for loop that creates and plots the four waves
            
                self.amplitude_jitter = random.uniform(0.5,1.0) #add amplitude jitter, for saturation, go above 1.0
                wave = self.create_wave(tstamps[i])

                if len(wave) == len(self.data)/4:

                    self.data[i::self.number_of_hydrophones] = wave  #storage variable to send data through ROS

                    n = len(wave)
                    t = np.arange(0,n*self.Ts,self.Ts)
                    if len(t) == len(wave):
                        ax[0].plot(t,wave)
                        ax[1].plot(t,wave)
                else:
                    wave = []


            if wave != [] and None not in self.data:
                wave = wave[n/2:n]  #cut out first half of signal (silence) to enhance FFT
                n = len(wave) # wave with half the number of data points

                k = np.arange(n)
                T = float(n)/float(self.Fs)            
                frq = k/T # two sides frequency range

                frq = frq[range(n/2)] # one side frequency range
                Y = np.fft.fft(wave)/n # fft computing and normalization
                Y = Y[range(n/2)]/2**self.resolution #linearize amplitude based on resolution of ADC
                Y = Y*(1/self.amplitude) #Compensate for FFT, multiply by inverse of amplitude
               
                #scale decided by ADC bits (resolution)           
                ax[0].set_title("Four Hydrophone Channels Full Scale")
                ax[0].set_ylim(0,2**self.resolution)
                ax[0].set_xlabel('Time')
                ax[0].set_ylabel('Amplitude')

                #zoomed in version of signal
                ax[1].set_title("Autosize on Hydrophone channels")
                ax[1].set_xlabel('Time')
                ax[1].set_ylabel('Amplitude')

                ax[2].cla()
                ax[2].set_title("FFT On Channel One")
                ax[2].plot(frq,abs(Y),'r') # plotting the FFT spectrum
                ax[2].set_xlim(5000,50000)
                ax[2].set_ylim(0,1)
                ax[2].set_xlabel('Freq (Hz)')
                ax[2].set_ylabel('|Y(freq)|')

                plt.pause(0.05)                

                self.data = list(map(int, self.data))
                #print self.data

                self.simulate_pub.publish(Ping(
                    header=Header(stamp=rospy.Time.now(),
                                  frame_id=self.frame),
                    channels=self.number_of_hydrophones,
                    samples=self.data_points,
                    data=self.data,
                    sample_rate=self.sample_rate*1000,
                    adc_bit=self.resolution,
                    actual_position=self.position))
            
            if self.signal_trigger == 'False':
                rate.sleep()
            if interrupted:
                break                

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