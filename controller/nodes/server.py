#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from controller.cfg import GainsConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {t_kp_x}, {t_kp_y}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("pd_controller_gains", anonymous = True)

    srv = Server(GainsConfig, callback)
    rospy.spin()
