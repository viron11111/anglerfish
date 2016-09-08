#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from controller.cfg import GainsConfig

def callback(config, level):
    rospy.loginfo("""pdx: {int_param}, pdy: {double_param},\ 
          {str_param}, {bool_param}, {size}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("pd_controller_gains", anonymous = True)

    srv = Server(GainsConfig, callback)
    rospy.spin()
