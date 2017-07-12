#!/usr/bin/env python
import rospy
import rosparam
import random
import numpy as np
import matplotlib.pyplot as plt

from std_msgs.msg import Header
from pinger_tracker.msg import *
from multilateration import Multilaterator, ReceiverArraySim, Pulse

#*********************************************************************
#  Using mix of uSec and sec, pay attention to units
#  Hydrophone locations are in mm, along with speed of sound

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

    def create_silence(self, offset):

        self.samples = ((self.signal_length/2)-offset)*self.sample_rate  #Number of samples during half the signal, for pre_signal
        #print self.samples

        pre_signal = [(2**self.resolution)/2.0]*int(self.samples)  #dead period prior to signal

        pre_signal = pre_signal + self.noise[0:len(pre_signal)]*random.uniform(1.0,2) #add noise with noise multiplier

        return pre_signal


    def create_wave(self, offset):
        #print ('%0.10f' % offset)
        self.Fs = self.sample_rate  # sampling rate
        self.Ts = 1.0/self.Fs # sampling interval

        self.samples = ((self.signal_length/2)-offset)*self.sample_rate  #Number of samples during a 400 uSec period, for pre_signal
        int_sample = self.samples
        int_sample = int(int_sample)
        #print self.samples

        pre_signal = self.create_silence(offset) #[(2**self.resolution)/2.0]*int(self.samples)  #dead period prior to signal


        t = np.arange(0.0,(self.signal_length/2)+offset,self.Ts) # time vector for signal waves
        #print ("self.signal_length: %f" % self.signal_length)
        #print ("len(t): %f" % len(t))
        #print ("self.samples: %f" % int_sample)
        total = len(t)+int_sample
        #print ("total: %f" % total)
        if total > self.signal_length/self.Ts:
            #print("too long")
            t = t[:len(t)-1]

        #print ("len(t) after: %f" % len(t))
        #print ('%0.8f' % offset)
        #print (self.signal_length/2)+offset
        #print len(t)+int(self.samples)

        ff = self.signal_freq   # frequency of the signal, 43 kHz for Anglerfish

        #self.phase_jitter randomly places phase within on sampling time
        #amplitude jitter for realism
        y = np.sin(2*np.pi*ff*t+self.phase_jitter)*(self.amplitude*self.amplitude_jitter) + 1  #create sine wave

        y = (y/2) * 2**self.resolution  #turn sine wave into an int

        y = np.array(y,dtype=int) #Help from Kevin, turn Floats in y to Int


        wave_func = np.append(pre_signal,y)  #append silence before signal to actual signal
        #print len(wave_func)

        return wave_func

    def get_pos(self, data):
        self.position = (data.x_pos, data.y_pos, data.z_pos)

    def __init__(self):
        rospy.init_node('signal_simulator')

        rospy.Subscriber('hydrophones/simulated_position', Transmitter_position, self.get_pos)

        self.simulate_pub = rospy.Publisher('hydrophones/ping', Ping, queue_size = 1)

        self.sample_rate = rospy.get_param('~sample_rate', 1000e3)  #ADC sampling rate
        #self.thresh = rospy.get_param('~thresh', 500)
        self.frame = rospy.get_param('~frame', '/hydrophones')
        #permute_str = rospy.get_param('~permute', '1 2 3 4')
        #self.samples = rospy.get_param('sample_number', 1024)
        self.resolution = rospy.get_param('resolution', 16)  #ADC bits
        self.signal_freq = rospy.get_param('signal_freq', 27e3)  #pinger freq
        self.amplitude = rospy.get_param('amplitude', 0.03)      #received signal amplitude 0.0-1.0
        self.number_of_hydrophones = rospy.get_param('number_of_hydrophones', 4)  
        self.signal_length = rospy.get_param('signal_length', 0.0008)  #800 uSec from default paul board

        self.signal_pub = rospy.Publisher('/hydrophones/ping', Ping, queue_size = 1)

        self.Fs = self.sample_rate  # sampling rate
        self.Ts = 1.0/self.Fs # sampling interval

        #count the number of published data point for assignment of empty self.data list
        self.data_points = int(self.signal_length/self.Ts)*self.number_of_hydrophones
        self.data = [None]*self.data_points
        
        #***********************************
        # position of ping, used in generation of time stamps
        # in (mm)s        
        self.position = (1000, 10000, -1000)  # in mm, default position        
        


        plt.ion()
        fig, ax = plt.subplots(3, 1)  #3x1 plot

        rate = rospy.Rate(1)  #rate of signals, 5 Hz for Anglerfish

        while not rospy.is_shutdown():

            tstamps = self.create_time_stamps(self.position)

            #converts timestamps to Sec because create_time_stamps uses uSec
            for i in range(0,4):
                tstamps[i] = tstamps[i]*10**-6
            #print self.Ts
            print tstamps

            #phase jitter, shifts sine wave left or right within one sampling period (1/300000 sec for Paul board)
            phase_jitter = ((1/self.sample_rate)/(1/self.signal_freq))*np.pi
            self.phase_jitter = random.uniform(-phase_jitter/2,phase_jitter/2)    
            
            ax[0].cla()
            ax[1].cla()

            #self.noise is used to add noise to the silent portion of the signal            
            self.noise = np.random.normal(-((2**self.resolution)*0.0005)/2,((2**self.resolution)*0.0005)/2,(int(self.signal_length/self.Ts)))
            
            #turn Float noise into Int noise
            for i in range(0,len(self.noise)):
                self.noise[i] = int(self.noise[i])
                        
            for i in range(0,4):  #for loop that creates and plots the four waves
            
                self.amplitude_jitter = random.uniform(0.5,1.0) #add amplitude jitter, for saturation, go above 1.0
                wave = self.create_wave(tstamps[i])
                #print len(wave)

                self.data[i::self.number_of_hydrophones] = wave  #storage variable to send data through ROS

                n = len(wave)
                t = np.arange(0,n*self.Ts,self.Ts)
                
                ax[0].plot(t,wave)
                ax[1].plot(t,wave)

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

            self.simulate_pub.publish(Ping(
                header=Header(stamp=rospy.Time.now(),
                              frame_id=self.frame),
                channels=self.number_of_hydrophones,
                samples=self.data_points,
                data=self.data,
                sample_rate=self.sample_rate,
                adc_bit=self.resolution))
            
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