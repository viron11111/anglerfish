#!/usr/bin/python

import serial
import numpy as np
import rospy
import geometry_msgs.msg 
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from std_msgs.msg import Float32
import binascii
import struct
import encodings
import math
import tf, tf2_ros


class Interface(object):

    def __init__(self):

        #self.pose_pub = rospy.Publisher("/static_point", PoseStamped, queue_size = 0)
        rospy.Subscriber("imu/data", Imu, self.stim300)
        rospy.Subscriber("imu/razor", Imu, self.razor)

    def stim300(self, data):

	    br = tf2_ros.TransformBroadcaster()
	    t = geometry_msgs.msg.TransformStamped()

	    t.header.stamp = rospy.Time.now()
	    t.header.frame_id = "world"
	    t.child_frame_id = "stim300"
	    t.transform.translation.x = 1.0
	    t.transform.translation.y = 1.0
	    t.transform.translation.z = 0.0
	    #q = tf.transformations.quaternion_from_euler(0, 0, msg.theta)
	    t.transform.rotation.x = data.orientation.x
	    t.transform.rotation.y = data.orientation.y
	    t.transform.rotation.z = data.orientation.z
	    t.transform.rotation.w = data.orientation.w

	    br.sendTransform(t)


        #br = tf2.TransformBroadcaster()
        #br.sendTransform((0, 0, 0), (data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w), rospy.Time.now(), "stim_imu", "world")
        #print data.orientation.x

    def razor(self, data):

	    br = tf2_ros.TransformBroadcaster()
	    t = geometry_msgs.msg.TransformStamped()

	    t.header.stamp = rospy.Time.now()
	    t.header.frame_id = "world"
	    t.child_frame_id = "razor"
	    t.transform.translation.x = 1.0
	    t.transform.translation.y = 0.5
	    t.transform.translation.z = 0.0
	    
	    #q = tf.transformations.euler_from_quaternion(data.orientation)
	    #print data.orientation
	    #tf.transform(data.orientation, )
	    t.transform.rotation.x = data.orientation.x
	    t.transform.rotation.y = data.orientation.y
	    t.transform.rotation.z = (data.orientation.z)#math.sqrt(0.5)
	    t.transform.rotation.w = (data.orientation.w)#math.sqrt(0.5)



	    br.sendTransform(t)


        #br = tf2.TransformBroadcaster()
        #br.sendTransform((0, 0, 0), (data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w), rospy.Time.now(), "stim_imu", "world")
        #print data.orientation.x        

def main():
	rospy.init_node('orientation_node', anonymous=False)

	Interface()

        try:
		rospy.spin()
        except rospy.ROSInterruptException:
		print "Shutting down"
                pass


if __name__ == '__main__':
	main() #sys.argv
