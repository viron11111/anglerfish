#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import math
#import sympy
#from sympy import nsolve,Symbol
#import mpmath
from itertools import combinations
import numpy.linalg as la

from scipy.optimize import fsolve

from multilateration import Multilaterator, ReceiverArraySim, Pulse
import random



class mission(object):

	def distance_to_time(self, pos):
		distance = math.sqrt((pos[0]-self.pinger_loc[0])**2+(pos[1]-self.pinger_loc[1])**2+(pos[2]-self.pinger_loc[2])**2)
		travel_time = distance / self.speed_of_sound
		return travel_time

	def time_difference(self):
		

		actual_ref = self.distance_to_time(ref)
		actual_b = self.distance_to_time(b)
		actual_c = self.distance_to_time(c)
		actual_d = self.distance_to_time(d)

		b_diff = (actual_b - actual_ref)*10**6
		c_diff = (actual_c - actual_ref)*10**6
		d_diff = (actual_d - actual_ref)*10**6

		return b_diff,c_diff,d_diff

	def getPulseLocation(self, timestamps, method=None):
		'''
		Returns a ros message with the location and time of emission of a pinger pulse.
		'''
		try:
			if method == None:
				method = self.method
			# print "\x1b[32mMultilateration algorithm:", method, "\x1b[0m"
			assert len(self.hydrophone_locations) == len(timestamps)
			source = None
			if method == 'bancroft':
				source = self.estimate_pos_bancroft(timestamps)
			elif method == 'LS':
				source = self.estimate_pos_LS(timestamps)
			else:
				print method, "is not an available multilateration algorithm"
				return
			response = SonarResponse()
			response.x = source[0]
			response.y = source[1]
			response.z = source[2]
			print "Reconstructed Pulse:\n\t" + "x: " + str(response.x) + " y: " + str(response.y) \
				+ " z: " + str(response.z) + " (mm)"
			return response
		except KeyboardInterrupt:
			print "Source localization interupted, returning all zeroes."
			response = SonarResponse()
			response.x, response.y, response.z = (0, 0, 0)

	def estimate_pos_LS(self, timestamps):
		'''
		Returns a ros message with the location and time of emission of a pinger pulse.
		'''
		self.timestamps = timestamps
		init_guess = np.random.normal(0, 100, 3)
		opt = {'disp': 0}
		opt_method = 'Powell'
		result = optimize.minimize(
			self.cost_LS, init_guess, method=opt_method, options=opt, tol=1e-15)
		if(result.success):
			source = [result.x[0], result.x[1], result.x[2]]
		else:
			source = [0, 0, 0]
		return source

	def cost_LS(self, potential_pulse):
		'''
		Compares the difference in observed and theoretical difference in time of arrival
		between tevery unique pair of hydrophones.

		Note: the result will be along the direction of the heading but not at the right distance.
		'''
		cost = 0
		t = self.timestamps
		c = self.c
		x = np.array(potential_pulse)
		for pair in self.pairs:
			h0 = self.hydrophone_locations[pair[0]]
			h1 = self.hydrophone_locations[pair[1]]
			residual = la.norm(x - h0) - la.norm(x - h1) - \
				c * (t[pair[0]] - t[pair[1]])
			cost += residual**2
		return cost

	def __init__(self):
		self.speed_of_sound = 1484000.0 #milimeters per second
		self.pinger_loc = (20000, 1000, -3000)  #milimeters

		self.hydrophone_locations = []
		#<!-- millimeters for greater accuracy -->
		hydrophone_locations = {   
		'hydro0': {'x':       0, 'y':       0, 'z':      0},
		'hydro1': {'x':   -25.4, 'y':       0, 'z':      100},
		'hydro2': {'x':    25.4, 'y':       0, 'z':      0},
		'hydro3': {'x':       0, 'y':   -25.4, 'z':      0}}

		for key in hydrophone_locations:
			sensor_location = np.array(
				[hydrophone_locations[key]['x'], hydrophone_locations[key]['y'], hydrophone_locations[key]['z']])
			self.hydrophone_locations += [sensor_location]

		#timestamps = self.time_difference(self.hydrophone_ref,self.hydrophone_b,self.hydrophone_c,self.hydrophone_d)
		
		#print self.time_diffs
		#print fsolve(self.find_position, (1))
		#x = Symbol('x')
		#y = Symbol('y')
		#ans1,ans2 = nsolve([x+y**2-4, x*y-3], [x, y], [1, 1])
		#self.find_position()
		#print self.timestamps
		#print timestamps

		'''c = 1.484  # millimeters/microsecond
		hydrophone_array = ReceiverArraySim(hydrophone_locations, c)
		sonar = Multilaterator(hydrophone_locations, c, 'LS')

		pulse = Pulse(-5251, -7620, 1470, 0)
		tstamps = hydrophone_array.listen(pulse)
		tstamps = tstamps - tstamps[0]
		res_msg = sonar.getPulseLocation(np.array(tstamps))'''
		#print tstamps

		    # pulses will be generated with inside a cube with side-length $(pulse_range) (mm)
        try:
            for h in range(3,4):
                # smallest cube will be a meter wide, largest will be 10 km wide
                pulse_range = 10**h  # in mm
                rand_args = [-pulse_range, pulse_range + 1]
                num_pulses = 10
                print "\n\x1b[1mGenerating " + str(num_pulses) + " pulses within a " \
                    + str(2*pulse_range/1000) + " meters wide cube\x1b[0m\n"

                #<!-- millimeters for greater accuracy -->
                hydrophone_locations = {   
                'hydro0': {'x':       0, 'y':       0, 'z':      0},
                'hydro1': {'x':   -25.4, 'y':       0, 'z':      0},
                'hydro2': {'x':    25.4, 'y':       0, 'z':      0},
                'hydro3': {'x':       0, 'y':   -25.4, 'z':      0}}

                c = 1.484  # millimeters/microsecond
                hydrophone_array = ReceiverArraySim(hydrophone_locations, c)
                sonar = Multilaterator(hydrophone_locations, c, 'LS')

                for i in range(num_pulses):
                    pulse = Pulse(random.randrange(*rand_args),
                                  random.randrange(*rand_args),
                                  random.randrange(*rand_args), 0)
                    print pulse
                    tstamps = hydrophone_array.listen(pulse)
                    tstamps = tstamps - tstamps[0]
                    print tstamps
                    
                    #print "Perfect timestamps: (microseconds)\n\t", tstamps
                    res_msg = sonar.getPulseLocation(np.array(tstamps))
                    #delete_last_lines(4)  # more concise output
                    res = np.array([res_msg[0], res_msg[1], res_msg[2]])
                    #print "\t\x1b[33m".ljust(22) + error(res, pulse.position()) + "\x1b[0m"
                    #print "Progressively adding noise to timestamps..."

                    '''for j in range(-5, 2):
                        sigma = 10**j
                        noisy_tstamps = [x + np.random.normal(0, sigma) for x in tstamps]
                        noisy_tstamps[0] = 0
                        #print "Noisy timestamps:\n\t", noisy_tstamps
                        res_msg = sonar.getPulseLocation(np.array(noisy_tstamps))
                        res = np.array([res_msg[0], res_msg[1], res_msg[2]])
                        #delete_last_lines(4)  # more concise output
                        #print "\t\x1b[33m" + ("sigma: " +  str(sigma)).ljust(16) \
                        #    + error(res, pulse.position()) + "\x1b[0m"'''
        except KeyboardInterrupt:
            print "\nAborting mutilateration tests prematurely"

def main():

	mission()

if __name__ == '__main__':
	main() #sys.argv

#self.hydrophone_ref = (0,0,0) #milimeters
		#self.hydrophone_b = (-100, 0,0)  #milimeters
		#self.hydrophone_c = (0,100,0)  #milimeters
		#self.hydrophone_d = (100,0.0,0)  #milimeters