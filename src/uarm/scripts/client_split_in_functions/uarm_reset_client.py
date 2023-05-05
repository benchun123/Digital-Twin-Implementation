#!/usr/bin/env python

from __future__ import print_function
import rospy
import uarm_msgs.msg
import actionlib

# Creates the SimpleActionClient, passing the type of the action
client2 = actionlib.SimpleActionClient('uarm_reset', uarm_msgs.msg.uarm_resetAction)

#
def reset_client():
    global client2
    try:                
        # Creates the SimpleActionClient, passing the type of the action
        # (FibonacciAction) to the constructor.
        
        print("connecting")
        # Waits until the action server has started up and started
        # listening for goals.
        client2.wait_for_server()
        print("connected")
        # Sends the goal to the action server.
        goal = uarm_msgs.msg.uarm_resetGoal()
        client2.send_goal(goal)
        print("goal sent")
        # Waits for the server to finish performing the action.
        print(client2.wait_for_result())
        print("result received")
        # Prints out the result of executing the action
        return client2.get_result()
    except:
        print("error with client2 connecting")
        return False

if __name__ == "__main__":
    rospy.init_node('uarm_reset_client', anonymous=True)
    try:
        while True:
            input("Enter anything for reset - nothing to exit")
            print(reset_client())
    except:
        pass
