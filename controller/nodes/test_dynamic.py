#!/usr/bin/env python

PACKAGE = 'controller'
import roslib;roslib.load_manifest(PACKAGE)
import rospy

import dynamic_reconfigure.client

def callback(config):
    rospy.loginfo("Config set to {t_kp_x}, {t_kp_y}".format(**config))

if __name__ == "__main__":
    rospy.init_node("dynamic_client")

    client = dynamic_reconfigure.client.Client("controller", timeout=30, config_callback=callback)

    r = rospy.Rate(5)
    while not rospy.is_shutdown():
        client.update_configuration()
        r.sleep()
