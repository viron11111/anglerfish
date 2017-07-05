#!/usr/bin/env python
import rospy
import rosparam

import numpy as np
from scipy import optimize

from sub8_msgs.srv import Sonar, SonarResponse

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from std_msgs.msg import UInt16


print "Waiting for sonar msg"
rospy.init_node('ping_plotter')
rospy.Subscriber('/hydrophones/ping/data', UInt16)

print "done"


'''
sonar = rospy.ServiceProxy('~/sonar/get_pinger_pulse', Sonar)
print "sonar serv proxy created"

try:
    while(True):
        try:
            pinger_pose = sonar()
        except Exception:
            plt.close('all')
            print "\nSonar driver is down, shutting down plotter"
            break
        print "x:", str(pinger_pose.x).rjust(15), "y:", str(pinger_pose.y).rjust(15), "z:", str(pinger_pose.z).rjust(15)
        ax.scatter(pinger_pose.x, pinger_pose.y, pinger_pose.z)
        plt.draw()
        plt.pause(0.1)
except KeyboardInterrupt:
    plt.close('all')
    print "\nshutting down plotter"'''