#!/usr/bin/env python


sampling_frequency = 100000
signal_frequency = 43000
distance_between_hydrophones = 0.05 #meters
speed_of_sound = 1484 #meters per second

#0.5 assuming signal can only be detected on peaks and troughs
wave_detection_threshold = 0.5

time_signal_below_threshold = wave_detection_threshold*(1/signal_frequency)

print time_signal_below_threshold