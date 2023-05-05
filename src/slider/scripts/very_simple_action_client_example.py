#!/usr/bin/env python3

import rospy
from uarm_msgs.srv import *
from std_srvs.srv import *
import uarm_msgs.msg
import actionlib
import time


#################################
#                               #
#       valid positions         #
#                               #
#      y element [-470, 0]      #
#                               #
#################################

#create ROS action client:
#CLIENT-NAME = actionlib.SimpleActionClient(TOPIC NAME, TYPE OF MESSAGE)
client = actionlib.SimpleActionClient('slider1/slider_move', uarm_msgs.msg.uarm_moveAction)

#start node
rospy.init_node('slider1_client', anonymous=True)

def move_client(y):
    
    print("connecting")
    # Waits until the action server has started up and started
    # listening for goals.        
    client.wait_for_server()
    print("connected")
        
    # Creates a goal to send to the action server.
    goal = uarm_msgs.msg.uarm_moveGoal(x=0,y=y,z=0)
    print("goal defined")
        
    # Sends the goal to the action server.
    client.send_goal(goal)
    print("goal sent")
        
    # Waits for the server to finish performing the action.
    client.wait_for_result()
        
    #gets the result of the task
    return client.get_result()


#########################################################
#      example for using the defined functions          #
#########################################################
    
#call move function and print result
print(move_client(-470))

time.sleep(1)

#call move function and print result
print(move_client(0))
