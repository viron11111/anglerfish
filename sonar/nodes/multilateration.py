#!/usr/bin/python
from __future__ import division
import numpy as np
from scipy import optimize
from itertools import combinations
import multilateration as mlat
import math
from pinger_tracker.msg import *
import rospy
from std_msgs.msg import Header
from time_signal_1d import TimeSignal1D

class Multilaterator(object):
    '''
    This class uses difference in time of arrival (DTOA) measurements to calculate the relative
    heading or postion of a signal source with respect to a receiver array.

    There as of now, there are 3 different solvers that you can use. You can select using the 'method'
    argument.
    bancroft - analytical solution, fast but not very numerically stable
    LS - optimization routine, fast and robust to noise
    LS1 - optimization routine, slow, but may be more accurate

    If the distance between receivers is orders of magnitude below the distance between the source
    and the array, the resulting position estimate should be in the right heading direction but with
    unreliable range. Otherwise, it may be possible to fully estimate the relative position of the
    pinger.
    '''
    def __init__(self, receiver_locations, c, method, init_guess):  # speed in millimeters/microsecond
        '''
        receiver_locations - N x 3 numpy array with the locations of receivers in units of millimeters.
            The location of the reference hydrophone is considered the origin of the receiver array's
            reference frame and should not be included
        c - speed at which the pulse propagates through the medium (millimeters / microsecond)
        method - kind of solver used to estimate the position or heading to the pulse
        '''

        #self.init_guess = np.random.normal(0,100,3)

        self.init_guess = init_guess
        self.receiver_locations = receiver_locations
        self.receiver_locations[1, 0]
        self.n = len(receiver_locations)
        self.pairs = list(combinations(range(self.n), 2))
        self.c = c
        self.method = method
        self.solvers = {'bancroft' : self.estimate_pos_bancroft,
                        'LS'       : lambda dtoa: self.estimate_pos_LS(dtoa, self.cost_LS),
                        'LS1'      : lambda dtoa: self.estimate_pos_LS1(dtoa, self.cost_LS)}

    def get_pulse_location(self, dtoa, method=None):
        '''
        Returns a 3-element list with the  coordinates  of the estimated position of a point source
        transmitter in the frame of the receiver array.

        timestamps - list of n-1 time dtoas, all with respect to a reference receiver
        '''        
        if method == None:
            method = self.method
        if not len(self.receiver_locations) == len(dtoa):
            raise RuntimeError('Number of non-reference receivers and dtoa measurents don\'t match')
        return self.solvers[method](dtoa)

    def estimate_pos_bancroft(self, dtoa):
        '''
        Uses the Bancroft Algorithm to solve for the position of a source base on dtoa measurements
        '''
        N = len(dtoa)
        if N < 4:
            raise RuntimeError('At least 4 dtoa measurements are needed')

        L = lambda a, b: a[0]*b[0] + a[1]*b[1] + a[2]*b[2] - a[3]*b[3]

        def get_B(delta):
            B = np.zeros((N, 4))
            for i in xrange(N):
                B[i] = np.concatenate([self.receiver_locations[i]/(self.c), [-dtoa[i]]]) + delta
            return B

        delta = min([.1 * np.random.randn(4) for i in xrange(10)],
                    key=lambda delta: np.linalg.cond(get_B(delta)))
        # delta = np.zeros(4) # gives very good heading for noisy timestamps,
        # although range is completely unreliable

        B = get_B(delta)
        a = np.array([0.5 * L(B[i], B[i]) for i in xrange(N)])
        e = np.ones(N)

        Bpe = np.linalg.lstsq(B, e)[0]
        Bpa = np.linalg.lstsq(B, a)[0]

        Lambdas = quadratic(
            L(Bpe, Bpe),
            2 * (L(Bpa, Bpe) - 1),
            L(Bpa, Bpa))
        if not Lambdas:
            return [0, 0, 0]

        res = []
        for Lambda in Lambdas:
            u = Bpa + Lambda * Bpe
            position = u[:3] - delta[:3]
            time = u[3] + delta[3]
            if any(dtoa[i] < time for i in xrange(N)): continue
            res.append(position*self.c)
        if len(res) == 1:
            source = res[0]
        elif len(res) == 2:
            # Assume that the source is below us
            source = [x for x in res if x[2] < 0]
            if not source:
                source = res[0]
            else:
                source = source[0]
        else:
            source = [0, 0, 0]

        return source

    def estimate_pos_LS(self, dtoa, cost_func):
        '''
        Uses the a minimization routine to solve for the position of a source base on dtoa measurements
        '''
        self.dtoa = dtoa 
        #rospy.logwarn(init_guess)
        opt = {'disp': 0}
        opt_method = 'Powell'
        #rospy.logwarn(self.init_guess)
        result = optimize.minimize(cost_func, self.init_guess, method=opt_method, options=opt, tol=1e-15)
        if(result.success):
            source = [result.x[0], result.x[1], result.x[2]]
        else:
            source = [0, 0, 0]
        return source

    def cost_LS(self, potential_pulse):
        """
        Slightly less accurate than the one below in terms of heading but much faster.
        """
        cost = 0
        t = self.dtoa
        x = potential_pulse[0]
        y = potential_pulse[1]
        z = potential_pulse[2]
        d0 = np.sqrt((x)**2 + (y)**2 + (z)**2)
        for i in range(self.n - 1):
            xi = self.receiver_locations[i, 0]
            yi = self.receiver_locations[i, 1]
            zi = self.receiver_locations[i, 2]
            di = np.linalg.norm([xi - x, yi - y, zi -z])
            receiver_i_cost = (di - d0 - self.c * t[i])**2
            cost = cost + receiver_i_cost
        return cost

    def cost_LS1(self, potential_pulse):
        '''
        Generates cost proportional to the difference in observed and theoretical dtoa measurements
        between every unique pair of receivers.
        '''
        cost = 0
        t = self.dtoa
        c = self.c
        x = np.array(potential_pulse)
        rcv = np.concatenate((np.array([[0, 0, 0]]), self.receiver_locations))

        for pair in self.pairs:
            cost += (np.linalg.norm(x-rcv[pair[0]]) - np.linalg.norm(x-rcv[pair[1]])
                     - c*(t[pair[0]] - t[pair[1]])) ** 2
        return cost

def get_time_delta(ref, non_ref):
    '''
    Given two signals that are identical except for a time delay and some noise,
    this will return the time delay of the non_ref signal with respect to ref

    ref - instance of TimeSignal1D
    non_ref - instance of TimeSignal1D

    returns: scalar (of the same type and units as an element of t)
    delta_t - time delay of non_ref w.r.t. ref
    cross_corr - cross-correlation of non_ref with ref
    t_corr - time delay values corresponding to the cross-correlation
    '''
    #print ref.samples
    if not isinstance(ref, mlat.TimeSignal1D) or not isinstance(non_ref, mlat.TimeSignal1D):
        raise TypeError("signals must be insstances of TimeSignal1D")
    if not np.isclose(ref.sampling_freq, non_ref.sampling_freq):
        raise RuntimeError("signals must have the same sampling frequency")

    cross_corr = np.correlate(non_ref.samples, ref.samples, mode='full')

    t_corr = np.linspace(-ref.duration(), non_ref.duration(), len(cross_corr))
    
    max_idx = cross_corr.argmax()
    signal_start_diff = non_ref.start_time - ref.start_time
    print signal_start_diff
    delta_t = t_corr[max_idx] + signal_start_diff

    cross_corr = mlat.TimeSignal1D(cross_corr, sampling_freq=ref.sampling_freq,
                                   start_time=-ref.duration() + signal_start_diff)

    return delta_t, cross_corr

def get_dtoas(ref_signal, non_ref_signals):
    '''
    Returns difference in time of arrival measurements of non_ref observations of a ref signal.
    ref_signal - TimeSignal1D
    non_ref_signals - iterable of TimeSignal1D's
    '''
    
    dtoas = []
    cross_corrs = []

    for delayed in non_ref_signals:
        ret = get_time_delta(ref=ref_signal, non_ref=delayed)
        
        dtoas.append(ret[0])
        cross_corrs.append(ret[1])

    return dtoas, cross_corrs

def quadratic(a, b, c):
    '''
    Solves a quadratic equation of the form ax^2 + bx + c = 0

    returns - list of solutions to the equation if there are any
    '''
    discriminant = b*b - 4*a*c
    if discriminant >= 0:
        first_times_a = (-b+np.copysign(np.sqrt(discriminant), -b))/2
        return [first_times_a/a, c/first_times_a]
    else:
        return []

def max_delta_t(c, receiver0, receiver1):
    '''
    Returns the absolute value of the maximum possible difference in time of arrival (DTOA)
    of a signal originating from a single source to two different receivers

    Note: all positions are 3D or 2D numpy arrays encoding positions in the same frame.
    @param c Speed of the signal in the medium. (Distance unit must be consistent with positions)
    @param receiver0 Position of the first receiver.
    @param receiver1 Position of the second receiver.

    returns: positive float (speed with the same distance and time units as the arguments)

    Note: Units of distance must match for both the position and speed arguments
    '''
    dist = np.linalg.norm(np.array(receiver1) - np.array(receiver0))
    return dist / c

def generate_dtoa(c, source_position, non_ref_positions, ref_position=[0, 0, 0]):
    '''
    Generates a difference in time of arrival for a signal emmited from a point source to a
    set of receivers. A difference is calculated between each of the non-reference receivers
    and the reference receiver.

    Note: all positions are 3D or 2D numpy arrays encoding positions in the same frame.
    @param c Speed of the signal in the medium.
    @param source_position Position of the source of the signal being emmitted.
    @param non_ref_positions List of non-reference receiver positions.
    @param ref_position Position of the reference receiver.
    '''
    ref_delta = np.linalg.norm(np.array(source_position) - np.array(ref_position)) / c
    non_ref_deltas = [np.linalg.norm(np.array(source_position) - np.array(p)) / c for p in non_ref_positions]
    return [nrd - ref_delta for nrd in non_ref_deltas]


class ReceiverArraySim(object):
    """
        Simulates an array of receivers that listens to point sources and returns the DTOA.
        (difference in time of arrival)
        Base Units:
            time   - microseconds
            length - millimeters
    """

    def __init__(self, hydrophone_locations, wave_propagation_speed_mps):
        self.c = wave_propagation_speed_mps
        self.hydrophone_locations = np.array([0, 0, 0])
        for key in hydrophone_locations:
            sensor_location = np.array(
                [hydrophone_locations[key]['x'], hydrophone_locations[key]['y'], hydrophone_locations[key]['z']])
            self.hydrophone_locations = np.vstack(
                (self.hydrophone_locations, sensor_location))
        self.hydrophone_locations = self.hydrophone_locations[1:]


    def listen(self, pulse):
        timestamps = []
        for idx in range(4):
            src_range = np.sqrt(
                sum(np.square(pulse.position() - self.hydrophone_locations[idx])))
            timestamps += [pulse.t + src_range / self.c]
        return np.array(timestamps)


class Pulse(object):
    """
    Represents an omnidirectional wave or impulse emmited from a point source
    """

    def __init__(self, x, y, z, t):
        self.x = x
        self.y = y
        self.z = z
        self.t = t

    def position(self):
        return np.array([self.x, self.y, self.z])

    def __repr__(self):
        return "Pulse:\t" + "x: " + str(self.x) + " y: " + str(self.y) + " z: " \
            + str(self.z) + " (mm)"


def quadratic(a, b, c):
    discriminant = b * b - 4 * a * c
    if discriminant >= 0:
        first_times_a = (-b + math.copysign(math.sqrt(discriminant), -b)) / 2
        return [first_times_a / a, c / first_times_a]
    else:
        return []    