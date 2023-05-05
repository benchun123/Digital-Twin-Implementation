#!/usr/bin/env python

import rospy
import uarm_msgs.msg
from uarm_msgs.srv import *
from std_srvs.srv import *
import actionlib
import time

##################################################################
#               methods that call the services                   #
##################################################################

#call uarm_move service

#################################
#                               #
#       valid positions         #
#                               #
#      x element [30, 280]      #
#                               #
#      y element [0, 280]       #
#                               #
#      z element [0, 130]       #
#                               #
#      w element [0, 180]       #
#                               #
#  RADIUS in x-y Ebene > 150    #
#                               #
#################################

class uArm_Remote_Client():

    def __init__(self,ns):
    
        self.name = ns
        
        #create ROS action client:
        #self.CLIENT-NAME = actionlib.SimpleActionClient(TOPIC NAME, TYPE OF MESSAGE)
        self.client = actionlib.SimpleActionClient(str(self.name+'uarm_move'), uarm_msgs.msg.uarm_moveAction)
        self.ranges = '''
#################################
#                               #
#       valid positions         #
#                               #
#      x element [30, 280]      #
#                               #
#      y element [-280, 280]    #
#                               #
#      z element [-120, 130]    #
#                               #
#      w element [0, 180]       #
#                               #
#  RADIUS in x-y Ebene > 150    #
#                               #
#  ATTENTION: Platform at z=0   #
#                               #
#################################'''
        print(self.ranges)

    def move_client(self, x, y, z, w):
    
        print("connecting")
        # Waits until the action server has started up and started
        # listening for goals.        
        self.client.wait_for_server()
        print("connected")
        
        # Creates a goal to send to the action server.
        goal = uarm_msgs.msg.uarm_moveGoal(x=x,y=y,z=z)
        print("goal defined")
        
        # Sends the goal to the action server.
        self.client.send_goal(goal)
        print("goal sent")
        
        # Waits for the server to finish performing the action.
        self.client.wait_for_result()
        
        # Prints out the result of executing the action
        return self.client.get_result()


    def move(self):
        print("select coordinates of target position")
        try:
            x = int(input("x: "))
            y = int(input("y: "))
            z = int(input("z: "))
            w = int(input("w: "))
            return self.move_client(x, y, z, w)
        except:
            return False

if __name__ == "__main__":

    #start node
    rospy.init_node('uarm_client', anonymous=True)
    
    #look for namespace in case of start with launchfile
    try:
        ns = rospy.get_param('~ns_client')
    except:
        ns = "uarm1/"
        
    #create controller object
    remote = uArm_Remote_Client(ns)
    
    #call move function and print result
    print(remote.move())
