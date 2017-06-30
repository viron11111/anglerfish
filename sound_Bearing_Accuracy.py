#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import math
import sympy
from sympy import nsolve,Symbol
import mpmath
from itertools import combinations
import numpy.linalg as la

from scipy.optimize import fsolve



class mission(object):

	def distance_to_time(self, pos):
		distance = math.sqrt((pos[0]-self.pinger_loc[0])**2+(pos[1]-self.pinger_loc[1])**2+(pos[2]-self.pinger_loc[2])**2)
		travel_time = distance / self.speed_of_sound
		return travel_time

	def time_difference(self, ref, b, c, d):
		actual_ref = self.distance_to_time(ref)
		actual_b = self.distance_to_time(b)
		actual_c = self.distance_to_time(c)
		actual_d = self.distance_to_time(d)

		b_diff = (actual_b - actual_ref)*10**6
		c_diff = (actual_c - actual_ref)*10**6
		d_diff = (actual_d - actual_ref)*10**6

		return b_diff,c_diff,d_diff

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

	def __init__(self):
		self.speed_of_sound = 1484000.0 #milimeters per second
		self.pinger_loc = (20000, 1000, -3000)  #milimeters

		self.hydrophone_ref = (0,0,0) #milimeters
		self.hydrophone_b = (-100, 0,0)  #milimeters
		self.hydrophone_c = (0,100,0)  #milimeters
		self.hydrophone_d = (100,0.0,0)  #milimeters

		#<!-- millimeters for greater accuracy -->
		#{   hydro0: {x:       0, y:       0, z:      0},
		#hydro1: {x:   -25.4, y:       0, z:   25.4},
		#hydro2: {x:    25.4, y:       0, z:      0},
		#hydro3: {x:       0, y:   -25.4, z:      0} }

		self.timestamps = self.time_difference(self.hydrophone_ref,self.hydrophone_b,self.hydrophone_c,self.hydrophone_d)
		
		#print self.time_diffs
		#print fsolve(self.find_position, (1))
		#x = Symbol('x')
		#y = Symbol('y')
		#ans1,ans2 = nsolve([x+y**2-4, x*y-3], [x, y], [1, 1])
		#self.find_position()
		print self.timestamps

def main():

	mission()

if __name__ == '__main__':
	main() #sys.argv
