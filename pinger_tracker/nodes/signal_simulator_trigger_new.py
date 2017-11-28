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
from advantech_pci1714.msg import *

import signal
import math

import os
import time
#*********************************************************************
#  Using mix of uSec and sec, pay attention to units
#  Hydrophone locations are in mm, along with speed of sound

class simulator():

    def position_service(self, data):
        return Actual_positionResponse(self.position)    

    def location_service(self, data):

        #can change x1, x2, x3, y2, y3
 
        #equilateral triangle layout
        '''hyp = 50.8
        var_a = hyp/2
        hydro0_xyz = [0,      0,     0]
        hydro1_xyz = [2*var_a,   0,     0]
        hydro2_xyz = [-var_a,  -var_a*np.sqrt(3),     0]
        hydro3_xyz = [-var_a,  var_a*np.sqrt(3), 0]'''

        #MIL T-shape layout
        #hydro0_xyz = [0,      0,     0]
        #hydro1_xyz = [100,   0,     0]
        #hydro2_xyz = [-100,  0,     0]
        #hydro3_xyz = [0,  -100, 0]  

        # Equilateral layout (actual)
        #hydro0_xyz = [0,      0,     0]
        #hydro1_xyz = [100.0,   0,     0]
        #hydro2_xyz = [-50,  86.6,     0]
        #hydro3_xyz = [-50,  -86.6, 0]

        # Equilateral layout (actual)
        #hydro0_xyz = [0,      0,     0]
        #hydro1_xyz = [173.2,   0,     0]
        #hydro2_xyz = [86.6,  150,     0]
        #hydro3_xyz = [86.6,  50, 0]

        #experimental layout
        hydro0_xyz = [0,      0,     0]
        hydro1_xyz = [173.2,   0,     0]
        hydro2_xyz = [86.6,  150,     0]
        hydro3_xyz = [86.6,  50, -100] 

        return Hydrophone_locations_serviceResponse(hydro0_xyz, hydro1_xyz, hydro2_xyz ,hydro3_xyz)    

    def sampling_rate(self,data):
        return Sample_rateResponse(self.sample_rate)

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

    def create_silence(self, offset):

        self.samples = ((self.signal_length/2)-offset)*self.sample_rate*1000  #Number of samples during half the signal, for pre_signal


        pre_signal = [(2**self.resolution)/2.0]*int(self.samples)  #dead period prior to signal

        if len(pre_signal) == len(self.noise[0:len(pre_signal)]):
            pre_signal = pre_signal #+ self.noise[0:len(pre_signal)]*random.uniform(1.0,2) #add noise with noise multiplier

        return pre_signal


    def create_wave(self, offset):

        offset = offset/10e5

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
        
        hydro = rospy.ServiceProxy('hydrophones/hydrophone_position', Hydrophone_locations_service)
        data = hydro()

        self.hydro0 = data.hydro0_xyz
        self.hydro1 = data.hydro1_xyz
        self.hydro2 = data.hydro2_xyz
        self.hydro3 = data.hydro3_xyz

        hydrophone_locations = {   
        'hydro0': {'x':  self.hydro0[0], 'y':   self.hydro0[1], 'z':  self.hydro0[2]},
        'hydro1': {'x':  self.hydro1[0], 'y':   self.hydro1[1], 'z':  self.hydro1[2]},
        'hydro2': {'x':  self.hydro2[0], 'y':   self.hydro2[1], 'z':  self.hydro2[2]},
        'hydro3': {'x':  self.hydro3[0], 'y':   self.hydro3[1], 'z':  self.hydro3[2]}}

        c = 1.484  # millimeters/microsecond
        hydrophone_array = ReceiverArraySim(hydrophone_locations, c)
        #sonar = Multilaterator(hydrophone_locations, c, 'LS')       
        
        pos = rospy.ServiceProxy('hydrophones/actual_position', Actual_position)
        pos = pos()
        self.position = pos.actual_position         

        print self.position

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
        
    def ping_service(self):

        #rospy.loginfo("start")

        sr = rospy.ServiceProxy('hydrophones/sample_rate', Sample_rate)
        sr = sr()
        self.sample_rate = sr.sample_rate
        #print self.sample_rate

        self.sample_rate = 2000
       
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

        print tstamps

        self.tstamps_pub = rospy.Publisher("/hydrophones/actual_time_stamps", Actual_time_stamps, queue_size = 1)

        self.tstamps_pub.publish(Actual_time_stamps(
                header=Header(stamp=rospy.Time.now(),
                              frame_id='signal_simulator'),
                actual_time_stamps=tstamps))       

      
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
        
            self.amplitude_jitter = 1.0 #random.uniform(0.5,1.0) #add amplitude jitter, for saturation, go above 1.0
            wave = self.create_wave(tstamps[i])
     
            if len(wave) == len(self.data)/4:

                self.data[i::self.number_of_hydrophones] = wave  #storage variable to send data through ROS
            else:
                wave = []     

        #if wave != [] and None not in self.data:   

        self.data = list(map(int, self.data))

        self.simulate_pub = rospy.Publisher("/hydrophones/pingraw", Pingdata, queue_size = 1)

        fivev = [(x-(65536.0/2.0))/4096 for x in self.data]
        #print max(fivev)

        self.simulate_pub.publish(Pingdata(
                header=Header(stamp=rospy.Time.now(),
                              frame_id='signal_simulator'),
                channels=4,
                samples=self.data_points,
                data=fivev,
                adc_bit = 12,
                sample_rate=self.sample_rate*1000)) 

        return Ping_serviceResponse(
        self.number_of_hydrophones,
            self.data_points,
            self.sample_rate*1000,
            self.resolution,
            self.data)           


    def calculate_error(self, x, y, z):

        self.position = [x,y,z]

        ref2 = rospy.ServiceProxy('/hydrophones/actual_position', Actual_position)
        ref2 = ref2()

        self.actual_x = ref2.actual_position[0]
        self.actual_y = ref2.actual_position[1]
        self.actual_z = ref2.actual_position[2]  


    def plot_grid_graph(self,x_list,y_list,z,z_list,typemeasure):
        # define grid.

        xi = np.linspace(-self.max_range/1000, self.max_range/1000, (self.max_range/1000)*2)
        yi = np.linspace(-self.max_range/1000, self.max_range/1000, (self.max_range/1000)*2)
        npts = len(x_list)

        x_list = [x / 1000 for x in x_list]
        y_list = [y / 1000 for y in y_list]

        # grid the data.
        #print "x_list: %i y_list: %i xi: %i yi: %i" %(len(x_list), len(y_list), len(xi), len(yi))
        zi = griddata(x_list, y_list, z_list, xi, yi, interp='nn')
        # contour the gridded data, plotting dots at the nonuniform data points.
        if typemeasure == 'Heading':
            levels = [0,0.05,0.1,0.15,0.2,0.25,0.3,0.35,0.4,0.45,0.5,0.55,0.6,0.65,0.7,0.75,0.8,0.85,0.9,0.95,1.0,2.0, 3.0, 4.0, 5.0, 6.0, 6.28]
        elif typemeasure == "Declination":
            levels = 15#[-1,0,0.05,0.1,0.15,0.2,0.25,0.3,0.35,0.4,0.45,0.5,0.75,1.0,6.30]
        CS = plt.contour(xi, yi, zi, 5, linewidths=0.5, colors='k')

        if typemeasure == 'Heading':
            #vmax = 1.04
            vmax = 6.28
            vmin = 0
        elif typemeasure == 'Declination':
            vmax=abs(zi).max()
            vmin=-abs(zi).max()

        CS = plt.contourf(xi, yi, zi, levels, #100,
                          vmax=vmax, vmin=vmin)
                          #vmax=abs(zi).max(), vmin=-abs(zi).max())

        cb = plt.colorbar()
        cb.set_label(label='%s Error (Radians)' % typemeasure)#,size=18)
        # plot data points.
        #plt.scatter(x, y, marker='o', s=5, zorder=10)
        ref = rospy.ServiceProxy('/hydrophones/hydrophone_position', Hydrophone_locations_service)
        ref = ref()

        plt.plot([ref.hydro0_xyz[0]/100, ref.hydro1_xyz[0]/100, ref.hydro2_xyz[0]/100, ref.hydro3_xyz[0]/100], 
            [ref.hydro0_xyz[1]/100, ref.hydro1_xyz[1]/100, ref.hydro2_xyz[1]/100, ref.hydro3_xyz[1]/100], 'wo')
        plt.plot([ref.hydro1_xyz[0]/100, ref.hydro2_xyz[0]/100, ref.hydro3_xyz[0]/100], 
            [ref.hydro1_xyz[1]/100, ref.hydro2_xyz[1]/100, ref.hydro3_xyz[1]/100], 'ko', markersize = 3)
        plt.plot([ref.hydro0_xyz[0]/100], [ref.hydro0_xyz[1]/100], 'yo', markersize = 3)

        plt.xlim(-self.max_range/1000, self.max_range/1000)
        plt.ylim(-self.max_range/1000, self.max_range/1000)
        z = abs(z/1000)
        plt.ylabel('Meters')#,size=18)
        plt.xlabel('Meters')#,size=18)
        figure_title = 'Pinger Location VS %s Accuracy' % typemeasure
        figure_sub_title = '%i k/S/s sample rate (%d points) at depth %i meter(s)' % (self.sample_rate,npts,z)
        

        plt.suptitle('%s\n%s' % (figure_title,figure_sub_title), weight = 'bold', size = 14, x = 0.46, y = 1.01, horizontalalignment='center')

        plt.savefig('Tshape_high_error_resolution_contours_%s_%i_d%i_s%i.png' % (typemeasure,self.sample_rate,z,npts), dpi=300,
                     orientation = 'landscape', bbox_inches='tight')
        plt.show()
        plt.clf()
        plt.close()              


    def __init__(self):

        rospy.init_node('signal_simulator_trigger')

        rospy.Service('hydrophones/sample_rate', Sample_rate, self.sampling_rate)
        rospy.Service('hydrophones/hydrophone_position', Hydrophone_locations_service, self.location_service)

        self.simulate_pub = rospy.Publisher("/hydrophones/pingraw", Pingdata, queue_size = 1)        

        self.position = [0, 0, 0]  # in mm, default position 

        self.sample_rate = rospy.get_param('~sample_rate', 2000)  #ADC sampling rate
        self.frame = rospy.get_param('~frame', '/hydrophones')
        self.resolution = rospy.get_param('resolution', 12)  #ADC bits
        self.signal_freq = rospy.get_param('signal_freq', 30)  #pinger freq
        self.amplitude = rospy.get_param('amplitude', 1.0)      #received signal amplitude 0.0-1.0
        self.number_of_hydrophones = rospy.get_param('number_of_hydrophones', 4)  
        self.signal_length = rospy.get_param('signal_length', 0.0008)  #800 uSec from default paul board

        #signal.signal(signal.SIGINT, self.signal_handler)
        srv = Server(SignalConfig, self.callback_signal)

        rospy.Subscriber('hydrophones/simulated_position', Transmitter_position, self.get_pos)
        rospy.Subscriber('hydrophones/signal_trigger', Bool, self.trigger_func)        

        self.simulate_pub = rospy.Publisher('hydrophones/ping', Ping, queue_size = 1)

        rospy.Service('hydrophones/ping_sim', Ping_service, self.ping_service)
        rospy.Service('/hydrophones/actual_time_stamps', Actual_time_stamps_service, self.actual_time_stamps_service)
        rospy.Service('hydrophones/actual_position', Actual_position, self.position_service)
        #rospy.Service('/hydrophones/hydrophone_locations', Hydrophone_locations_service, self.hydro_locations)

        self.sample_rate = 300
        self.signal_length = 0.0016

        rate = rospy.Rate(1)  #rate of signals, 5 Hz for Anglerfish

        self.actual_x = 0
        self.actual_y = 0
        self.actual_z = 0
        self.crane_x = 0
        self.crane_y = 0
        self.crane_z = 0

        self.W  = '\033[0m'  # white (normal)
        self.R  = '\033[31m' # red
        self.G  = '\033[32m' # green
        self.O  = '\033[43m' # orange
        self.B  = '\033[34m' # blue
        self.P  = '\033[35m' # purple      

        self.position = [3000.0, 3000.0, -1000.0]
        self.sample_rate = 1000

        self.head_error = 0
        self.declination_error = 0
        x_list = []
        y_list = []
        z_list = []
        d_list = []

        rate = rospy.Rate(1)  #rate of signals, 5 Hz for Anglerfish

        trigger = 0

        samples = 0
        self.heading_error_sum = 0.0
        self.declination_error_sum = 0.0
        self.distance_error_sum = 0.0

        resolution = 10000  

        self.xdistancemin = -10000 #in mm
        self.xdistancemax = 10000 #in mm
        self.ydistancemin =  -10000 #in mm
        self.ydistancemax = 10000 #inmm    

        #**********for polar coors****************

        self.sample_rate = 2000
        z = -1000 #depth of pinger

        self.max_range = 10000
        distance_resolution = 5000
        degree_angle_resolution = 1
        rad_resolution = math.radians(degree_angle_resolution)

        number_of_steps_per_rev = 360.0/degree_angle_resolution
        number_of_rings = self.max_range/distance_resolution
        total_samples = number_of_steps_per_rev * number_of_rings

        print rad_resolution
        print number_of_rings
        print "total samples %i" % total_samples

        for dis in range(distance_resolution,self.max_range+distance_resolution, distance_resolution):
            for deg in range(0,int(number_of_steps_per_rev)):
                phi = deg*rad_resolution
                x = dis * np.cos(phi)
                y = dis * np.sin(phi)
                print "x: %f y: %f" % (x,y)

                self.calculate_error(x,y,z)
                self.ping_service()

                x_list = x_list + [x]
                y_list = y_list + [y]
                z_list = z_list + [self.head_error]
                d_list = d_list + [self.declination_error]
                #print z_list
                time.sleep(0.5)


        self.plot_grid_graph(x_list,y_list,z,z_list,'Heading')
        self.plot_grid_graph(x_list,y_list,z,d_list,'Declination')       

        #****************polar coors**********************         

        os.system("rosnode kill acoustic_monte")            


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