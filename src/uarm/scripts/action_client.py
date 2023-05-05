#!/usr/bin/env python

from actionlib_msgs.msg import GoalStatusArray
import uarm_msgs.msg
import actionlib
import thread
import rospy
import time

class ActionClient():

    def __init__(self, action_name):
        self.action_name = action_name
        self.client = actionlib.SimpleActionClient(self.action_name, uarm_msgs.msg.uarm_moveAction)
        self.lis = rospy.Subscriber(str(self.action_name+'/status'), GoalStatusArray, self.callback)
        self.status = 0

    def callback(self, data):
        try:
            self.status = data.status_list[0].status
        except:
            self.status = 0

    def input_thread(self, a_list):
        raw_input()
        a_list.append(True)
    
    def process(self):
        a_list = []
        thread.start_new_thread(self.input_thread, (a_list,))
        while self.status == 1 and not a_list:
            pass

        if a_list:
            return True
        else:
            return False

    def move(self, x, y, z, w):
        try:                
            #print("connecting")
            # Waits until the action server has started up and started
            # listening for goals.
            self.client.wait_for_server()
            #print("connected")
            # Creates a goal to send to the action server.
            goal = uarm_msgs.msg.uarm_moveGoal(x=x,y=y,z=z, w=w)
            #print("goal defined")
            # Sends the goal to the action server.
            self.client.send_goal(goal)
            #print("goal sent")
            # Waits for the server to finish performing the action.
            time.sleep(.1)
            '''
            print("Press ENTER to interrupt")
            interrupted = self.process()

            if interrupted:
                self.client.cancel_all_goals()
                print("goal cancelled")
                return False
            '''
            self.client.wait_for_result()
            #print("result received")
            # Prints out the result of executing the action
            return self.client.get_result()
        except:
            return False

