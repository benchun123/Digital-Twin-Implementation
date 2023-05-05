#!/usr/bin/env python
import rospy
from uarm_msgs.srv import *
from std_srvs.srv import *
from std_msgs.msg import Float64
import uarm_msgs.msg
import uarm_msgs.srv
import time
import json
import actionlib
from interface import conveyor_sim_Server



#################################
#                               #
#          valid speeds         #
#                               #
#      y element [-40, 40]      #
#                               #
#################################

class Conveyor_Remote_Server():

    def __init__(self,ns):
        self.mutex = True
        self.name = ns
        self.reload_action_server = actionlib.SimpleActionServer(str(self.name+'reload'), uarm_msgs.msg.uarm_moveAction, self.reload_conveyor, auto_start = False)
        self.reload_action_server.start()
        self.set_speed_service = rospy.Service(str(self.name+'set_speed'), set_float, self.move)
        self.stop_service = rospy.Service(str(self.name+'stop'), set_float, self.move)
        self.conveyor = conveyor_sim_Server(self.name)
        print("uarm_listener started")

    def move(self,data):        
        if self.mutex:
            if self.conveyor.move(data.x):    
                return set_floatResponse(True)
            else:
                return set_floatResponse(False)
        else:
            return set_floatResponse(False)

    def stop(self):
        print("server")
        if self.conveyor.stop():
            TriggerResponse(success=True)
        else:
            TriggerResponse(success=False)

    def reload_conveyor(self,data):
        print("reloading")
        result = uarm_msgs.msg.uarm_moveResult()
        # api-endpoint
        self.conveyor.move(-40)
        if self.mutex:
            self.mutex = False
            while not self.conveyor.loaded:
                if self.reload_action_server.is_preempt_requested():
                    print("recieved cancle request")
                    result.success = False
                    self.mutex = True
                    self.conveyor.stop()
                    return self.reload_action_server.set_preempted()
            self.conveyor.move(0)
           
            if self.conveyor.loaded:
                result.success = True
                self.mutex = True
                
                print(result)
                return self.reload_action_server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('conveyor_gazebo_server', anonymous=True)
    try:
        ns = rospy.get_param('~ns')
    except:
        ns = "conveyor1/"
    server = Conveyor_Remote_Server(ns)
    rospy.spin()
