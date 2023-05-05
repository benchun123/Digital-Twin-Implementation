#!/usr/bin/env python

import rospy
from uarm_msgs.srv import *
from std_srvs.srv import *
from std_msgs.msg import Bool
from action_client import ActionClient
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

class Slider_Remote_Client():
    
    def __init__(self, ns):
        self.busy = True
        self.name = ns
        #self.client_move = ActionClient(str(self.name+'slider_move'))
        self.client_move = actionlib.SimpleActionClient(str(self.name+'slider_move'), uarm_msgs.msg.uarm_moveAction)
        self.client_limit = actionlib.SimpleActionClient(str(self.name+'drive_to_limit_switch'), uarm_msgs.msg.uarm_moveAction)
        self.status_listener = rospy.Subscriber(str(self.name+"busy"), Bool, self.callback)

    def callback(self,data):
        self.busy = data.data
    '''
    def move_client(self, y):
        if not self.busy:  
            return self.client_move.move(x=0, y=y, z=0)  
        else:
            return False
    '''
    def move_client(self, x = 0 , y = 0, z=0):
        try:                
            print("connecting")
            # Waits until the action server has started up and started
            # listening for goals.
            self.client_move.wait_for_server()
            print("connected")
            # Creates a goal to send to the action server.
            goal = uarm_msgs.msg.uarm_moveGoal(x=x,y=y,z=z)
            print("goal defined")
            # Sends the goal to the action server.
            self.client_move.send_goal(goal)
            print("goal sent")
            # Waits for the server to finish performing the action.
            time.sleep(.1)
            self.client_move.wait_for_result()
            # Prints out the result of executing the action
            result = self.client_move.get_result()

            print("result recieved")

            return result
        except:
            return False

    def limit_client(self, x = 0 , y = 0, z=0):
        try:                
            print("connecting")
            # Waits until the action server has started up and started
            # listening for goals.
            self.client_limit.wait_for_server()
            print("connected")
            # Creates a goal to send to the action server.
            goal = uarm_msgs.msg.uarm_moveGoal(x=x,y=y,z=z)
            print("goal defined")
            # Sends the goal to the action server.
            self.client_limit.send_goal(goal)
            print("goal sent")
            # Waits for the server to finish performing the action.
            time.sleep(.1)
            self.client_limit.wait_for_result()
            # Prints out the result of executing the action
            result = self.client_limit.get_result()

            print("result recieved")

            return result
        except:
            return False

    def stop():
        print("select pump state True or False")
        rospy.wait_for_service(str(self.name+'stop'))
        print("connected")
        try:
            service_object = rospy.ServiceProxy(str(self.name+'stop'), Trigger)
            received_response = service_object()
            print("service called")
            return received_response.success
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

if __name__ == "__main__":
    rospy.init_node('slider_move_client', anonymous=True)
    ns = ""
    try:
        ns = rospy.get_param('~ns_client')
    except:
        pass
    server = Slider_Remote_Client(ns)
    y = 0
    time.sleep(9)
    try:
        while not rospy.is_shutdown() and y != 9:

            if True:# server.client_move.status == 0:
                print("select target Position -470 to 0")
                try:
                    y = int(input("9 to end or y: "))
                except:
                    y = 0
                if y == 9:
                    break
                print(server.move_client(y=y))
            else:
                time.sleep(2)
                print("waiting for slider to be ready...")
    except KeyboardInterrupt:
        pass
