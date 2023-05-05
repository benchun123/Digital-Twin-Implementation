#!/usr/bin/env python

from __future__ import print_function
import rospy
import uarm_msgs.msg
import actionlib

# Creates the SimpleActionClient, passing the type of the action
client3 = actionlib.SimpleActionClient('uarm_auto_move', uarm_msgs.msg.uarm_moveAction)

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
#  RADIUS in x-y Ebene > 150    #
#                               #
#################################

def auto_move_client(x, y, z):
    global client3
    try:        
        print("connecting")
        # Waits until the action server has started up and started
        # listening for goals.
        client3.wait_for_server()
        print("connected")
        # Creates a goal to send to the action server.
        goal = uarm_msgs.msg.uarm_moveGoal(x=x,y=y,z=z)
        print("goal defined")
        # Sends the goal to the action server.
        client3.send_goal(goal)
        print("goal sent")
        # Waits for the server to finish performing the action.
        print(client3.wait_for_result())
        print("result received")
        # Prints out the result of executing the action
        return client3.get_result()  # A FibonacciResult
    except:
        return False 

if __name__ == "__main__":
    rospy.init_node('uarm_auto_move_client', anonymous=True)
    try:
        while True:
            print("select target Position")
            x = int(input("x: "))
            y = int(input("y: "))
            z = int(input("z: "))
            print(auto_move_client(x, y, z))
    except:
        pass
