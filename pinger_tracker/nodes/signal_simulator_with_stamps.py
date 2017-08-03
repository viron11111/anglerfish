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
from pinger_tracker.srv import *

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
        self.tx_rate = int("{signal_rate}".format(**config))
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

        #offset = offset/10e5
        offset = 0

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

    def hydro_locations(self, data):
        hydro0_xyz=self.hydro0
        hydro1_xyz=self.hydro1
        hydro2_xyz=self.hydro2
        hydro3_xyz=self.hydro3  
        return hydro0_xyz, hydro1_xyz, hydro2_xyz, hydro3_xyz

    def actual_time_stamps_service(self, data):
        hydrophone_locations = {   
        'hydro0': {'x':  self.hydro0[0], 'y':   self.hydro0[1], 'z':  self.hydro0[2]},
        'hydro1': {'x':  self.hydro1[0], 'y':   self.hydro1[1], 'z':  self.hydro1[2]},
        'hydro2': {'x':  self.hydro2[0], 'y':   self.hydro2[1], 'z':  self.hydro2[2]},
        'hydro3': {'x':  self.hydro3[0], 'y':   self.hydro3[1], 'z':  self.hydro3[2]}}

        c = 1.484  # millimeters/microsecond
        hydrophone_array = ReceiverArraySim(hydrophone_locations, c)
        #sonar = Multilaterator(hydrophone_locations, c, 'LS')

        pulse = Pulse(self.position[0], self.position[1], self.position[2], 0)
        tstamps = hydrophone_array.listen(pulse)
        tstamps = tstamps - tstamps[0]

        #converts timestamps to Sec because create_time_stamps uses uSec
        for i in range(0,4):
            tstamps[i] = tstamps[i]*10**-6

        self.tstamps=tstamps

        microseconds = [1e6,1e6,1e6,1e6]
        self.tstamps = [x * y for x, y in zip(self.tstamps,microseconds)]
        actual_time_stamps = list(self.tstamps)

        return Actual_time_stamps_serviceResponse(actual_time_stamps)

    def discrete_time_stamps(self,time):

        step = 1.0/(self.sample_rate)*1000

        time_holder = time#+np.random.uniform(-step/2, step/2)

        divider = abs(int(time_holder/(step)))
        min_time = abs(divider*step+2*step)
        
        mylist = np.arange(-min_time, min_time, step)

        minimum = min(mylist, key=lambda x:abs(x-time_holder))
 
        return minimum

        
    def ping_service(self, pinger_position):

        sr = rospy.ServiceProxy('hydrophones/sample_rate', Sample_rate)
        sr = sr()
        self.sample_rate = sr.sample_rate
        #print self.sample_rate
       
        self.Fs = self.sample_rate*1000  # sampling rate
        self.Ts = 1.0/self.Fs # sampling interval

        pos = rospy.ServiceProxy('hydrophones/actual_position', Actual_position)
        pos = pos()
        self.position = pos.actual_position

        #print self.position

        hydro = rospy.ServiceProxy('hydrophones/hydrophone_position', Hydrophone_locations_service)
        data = hydro()

        self.hydro0 = data.hydro0_xyz
        self.hydro1 = data.hydro1_xyz
        self.hydro2 = data.hydro2_xyz
        self.hydro3 = data.hydro3_xyz

        ref = rospy.ServiceProxy('/hydrophones/actual_time_stamps', Actual_time_stamps_service)
        timestamps = ref()
        tstamps = timestamps.actual_time_stamps

        tstamps = list(tstamps)
        #print tstamps

        for i in range(3):
            tstamps[i+1] = self.discrete_time_stamps(tstamps[i+1])
        #print tstamps
      
        phase_jitter = ((1.0/float(self.sample_rate*1000))/(1.0/(self.signal_freq*1000)))*np.pi
        self.phase_jitter = random.uniform(-phase_jitter/2,phase_jitter/2)             
        
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
            else:
                wave = []     

        #if wave != [] and None not in self.data:   

        self.data = list(map(int, self.data))

        return Ping_serviceResponse(self.number_of_hydrophones,
            self.data_points,
            self.sample_rate*1000,
            self.resolution,
            tstamps,
            self.data)           


    def __init__(self):

        rospy.init_node('signal_simulator_trigger')

        self.position = [0, 0, 0]  # in mm, default position 
        '''self.trigger = 'False'
        self.hydro0 = [0,     0,     0]
        self.hydro1 = [-25.4, 0,     0]
        self.hydro2 = [25.4,  0,     0]
        self.hydro3 = [0,     -25.4, 0] 
        self.tx_rate = 1.0'''

        self.sample_rate = rospy.get_param('~sample_rate', 600)  #ADC sampling rate
        self.frame = rospy.get_param('~frame', '/hydrophones')
        self.resolution = rospy.get_param('resolution', 16)  #ADC bits
        self.signal_freq = rospy.get_param('signal_freq', 27)  #pinger freq
        self.amplitude = rospy.get_param('amplitude', 0.1)      #received signal amplitude 0.0-1.0
        self.number_of_hydrophones = rospy.get_param('number_of_hydrophones', 4)  
        self.signal_length = rospy.get_param('signal_length', 0.0008)  #800 uSec from default paul board

        #signal.signal(signal.SIGINT, self.signal_handler)
        srv = Server(SignalConfig, self.callback_signal)

        rospy.Subscriber('hydrophones/simulated_position', Transmitter_position, self.get_pos)
        rospy.Subscriber('hydrophones/signal_trigger', Bool, self.trigger_func)        

        self.simulate_pub = rospy.Publisher('hydrophones/ping', Ping, queue_size = 1)

        rospy.Service('hydrophones/ping', Ping_service, self.ping_service)
        rospy.Service('/hydrophones/actual_time_stamps', Actual_time_stamps_service, self.actual_time_stamps_service)
        #rospy.Service('/hydrophones/hydrophone_locations', Hydrophone_locations_service, self.hydro_locations)

        self.sample_rate = 300
        self.signal_length = 0.0016

        rate = rospy.Rate(1)  #rate of signals, 5 Hz for Anglerfish

        while not rospy.is_shutdown():

            '''tstamps = self.create_time_stamps(self.position)

            for i in range(0,4):
                tstamps[i] = tstamps[i]*10**-6

            self.tstamps=tstamps

            microseconds = [1e6,1e6,1e6,1e6]
            self.tstamps = [x * y for x, y in zip(self.tstamps,microseconds)]
            self.tstamps = list(self.tstamps)

            #phase jitter, shifts sine wave left or right within one sampling period (1/300000 sec for Paul board)
            phase_jitter = ((1.0/float(self.sample_rate*1000))/(1.0/(self.signal_freq*1000)))*np.pi
            self.phase_jitter = random.uniform(-phase_jitter/2,phase_jitter/2)    

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
                else:
                    wave = []             

                self.data = list(map(int, self.data))
            
            if self.signal_trigger == 'False':
                
            if interrupted:
                break          '''
            rate.sleep()                


def main():
    rospy.init_node('signal_simulator_trigger', anonymous=False)

    simulator()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print "Shutting down"
        pass
    


if __name__ == '__main__':
    main()
