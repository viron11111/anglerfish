#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import math
import sympy
from sympy import nsolve,Symbol
import mpmath

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

		b_diff = actual_b - actual_ref
		c_diff = actual_c - actual_ref
		d_diff = actual_d - actual_ref

		return b_diff,c_diff,d_diff

	def find_position(self):

		#x = Symbol('x')
		#y = Symbol('y')
		z = Symbol('z')

		answer = nsolve(
			[(1/1484)*math.sqrt(-1.9**2+10**2+z**2)
		    -math.sqrt(-2**2+10**2+z**2)-0.0000012],
			[z], 
			[3.0])
		print answer

		#(1/self.speed_of_sound)*math.sqrt((-2-self.hydrophone_b[0])**2+(10-self.hydrophone_b[1])**2+(z-self.hydrophone_b[2])**2)-math.sqrt(-2**2+10**2+z**2)-self.time_diffs[0]
		#(1/self.speed_of_sound)*math.sqrt((-2-self.hydrophone_c[0])**2+(y-self.hydrophone_c[1])**2+(z-self.hydrophone_c[2])**2)-math.sqrt(-2**2+y**2+z**2)-self.time_diffs[1]
		#)
		#(1/self.speed_of_sound)*math.sqrt((-2-self.hydrophone_d[0])**2+(y-self.hydrophone_d[1])**2+(z-self.hydrophone_d[2])**2)-math.sqrt(-2**2+y**2+z**2)-self.time_diffs[2])
		


	def __init__(self):
		self.speed_of_sound = 1484.0 #meters per second
		self.pinger_loc = (-2, 10, 3)  #meters

		self.hydrophone_ref = (0,0,0) #meters
		self.hydrophone_b = (-0.1, 0,0)  #meters
		self.hydrophone_c = (0,0.1,0)  #meters
		self.hydrophone_d = (0.1,0.0,0)  #meters

		self.time_diffs = self.time_difference(self.hydrophone_ref,self.hydrophone_b,self.hydrophone_c,self.hydrophone_d)
		
		#print self.time_diffs
		#print fsolve(self.find_position, (1))
		#x = Symbol('x')
		#y = Symbol('y')
		#ans1,ans2 = nsolve([x+y**2-4, x*y-3], [x, y], [1, 1])
		self.find_position()
		print 'done'

def main():

	mission()

if __name__ == '__main__':
	main() #sys.argv

'''from scipy.optimize import fsolve
import math

def equations(p):
    x, y = p
    return (x+y**2-4, math.exp(x) + x*y - 3)

x, y =  fsolve(equations, (1, 1))

print equations((x, y))'''



'''sampling_frequency = 100000.0
signal_frequency = 43000.0
distance_between_hydrophones = 1 #meters


#0.5 assuming signal can only be detected on peaks and troughs
wave_detection_threshold = 0.5

time_between_samples = 1 / sampling_frequency
signal_time_period = 1 / signal_frequency
time_signal_below_threshold = wave_detection_threshold*(1/signal_frequency)

travel_time = distance_between_hydrophones / speed_of_sound

x = np.linspace(-9, 9, 40)
y = np.linspace(-5, 5, 40)
x, y = np.meshgrid(x, y)

def axes():
    plt.axhline(0, alpha=.1)
    plt.axvline(0, alpha=.1)

a = 1
axes()
plt.contour(x, y, (y**2 + 0.5*a*x), [0], colors='k')
plt.show()
'''