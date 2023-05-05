#! /usr/bin/env python

from __future__ import print_function
import rospy
from uarm_msgs.srv import *
from std_srvs.srv import *
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import Float64
from sensor_msgs.msg import Range
from std_msgs.msg import Bool
import uarm_msgs.msg
import time
import numpy as np
import actionlib

class conveyor_sim_Server(object):
    # create messages that are used to publish feedback/result
    

    def __init__(self, ns):
        self.ns = ns
        self.rolls = 135
        self.sensor = 0
        self.loaded = False
        self.lis = rospy.Subscriber(str(self.ns+"detector"), Range, self.callback)
        self.pub = rospy.Publisher(str(self.ns+'velocity/command'), Float64MultiArray, queue_size=1)
        self.pub_vel = rospy.Publisher(str(self.ns+'speed'), Float64, queue_size=1)
        self.pub_loaded = rospy.Publisher(str(self.ns+"loaded"), Bool, queue_size=1)
    	print("server initialized")

    def pub_joints(self, vel):
            mat = Float64MultiArray()
            mat.layout.dim.append(MultiArrayDimension())
            mat.layout.dim[0].label = ''
            mat.layout.dim[0].size = self.rolls
            mat.layout.dim[0].stride = 1
            mat.layout.data_offset = 0
            mat.data = [vel]*self.rolls
            self.pub.publish(mat)
            self.pub_vel.publish(vel)

    def callback(self, data):
        self.sensor = data.range
        if self.sensor > 0.1:
            self.loaded = False
        else:
            self.loaded = True
        self.pub_loaded.publish(self.loaded)
    
    def move(self, goal):
    	print("call recieved")
        # helper variables

        self.pub_joints(float(goal))
        return True

    def stop(self):
        print("interface")
        self.pub_joints(0)
        return True

        
if __name__ == '__main__':
    rospy.init_node('conveyor_gazebo_interface_node')
    server = conveyor_sim_Server(rospy.get_param('~ns'))
    rospy.spin()
