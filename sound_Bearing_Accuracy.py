#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import math

sampling_frequency = 100000.0
signal_frequency = 43000.0
distance_between_hydrophones = 1 #meters
speed_of_sound = 1484.0 #meters per second

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
