#!/usr/bin/env python
from genpy.message import Message
import rospy
from uarm_msgs.srv import *
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Float64
import uarm_msgs.msg
import time
import json
import actionlib
from real_interface import Slider_real_interface

#mutex = True
slider_pos = 0

#################################
#                               #
#       valid positions         #
#                               #
#      y element [-470, 0]      #
#                               #
#################################

class Slider_Remote_Server():

    def __init__(self, ns):
        self.ns = ns
        #in preparation for spawning with reference position in world
        '''
        self.x = float(rospy.get_param('~x'))
        self.y = float(rospy.get_param('~y'))
        self.z = float(rospy.get_param('~z'))
        self.w = float(rospy.get_param('~Y'))
        '''
        self.slider_move_action_server = actionlib.SimpleActionServer(str(self.ns+'slider_move'), uarm_msgs.msg.uarm_moveAction, self.move, auto_start = False)
        self.slider_move_action_server.start()
        self.slider_drive_to_limit_switch_action_server = actionlib.SimpleActionServer(str(self.ns+'drive_to_limit_switch'), uarm_msgs.msg.uarm_moveAction, self.drive_to_limit_switch, auto_start = False)
        self.slider_drive_to_limit_switch_action_server.start()
        self.stop_service = rospy.Service(str(self.ns+'stop'), Trigger, self.stop)
        self.shift_service = rospy.Service(str(self.ns+'set_shift'), set_int, self.set_shift)
        self.timeout_service = rospy.Service(str(self.ns+'set_timeout'), set_int, self.set_timeout)
        self.state_service = rospy.Service(str(self.ns+'state'), Trigger, self.state)
        
        self.slider = Slider_real_interface(ns = self.ns)
        #self.mutex = False
        #s7 = rospy.Subscriber("slider_pos", Float64, callback)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.loginfo(rospy.get_caller_id() + " " + self.ns + " server initialized")

    def move(self, data):
        result = uarm_msgs.msg.uarm_moveResult()
        # api-endpoint
        #self.state(5)
        #time.sleep(.5)

        if not self.slider.move(int(data.y)):
            #self.mutex = False
            rospy.loginfo(rospy.get_caller_id() + " " + self.ns + " target rejected")
            result.success = False
            return self.slider_move_action_server.set_succeeded(result)
        rospy.loginfo(rospy.get_caller_id() + " " + self.ns + " target accepted")
        while self.slider.busy:
            #self.state()
            if self.slider_move_action_server.is_preempt_requested():
                rospy.loginfo(rospy.get_caller_id() + " " + self.ns + " recieved cancle request")
                result.success = False
                self.slider.stop()
                return self.slider_move_action_server.set_preempted()
        
        if not self.slider.busy:
            result.success = True
            #self.mutex = True
            
            print(result)
            return self.slider_move_action_server.set_succeeded(result)

    def drive_to_limit_switch(self, data):
        result = uarm_msgs.msg.uarm_moveResult()
        # api-endpoint
        #self.state(5)
        #time.sleep(.5)

        if not self.slider.set_drive_to_limit():
            #self.mutex = False
            rospy.loginfo(rospy.get_caller_id() + " " + self.ns + " target rejected")
            result.success = False
            return self.slider_move_action_server.set_succeeded(result)
        rospy.loginfo(rospy.get_caller_id() + " " + self.ns + " target accepted")
        while self.slider.busy:
            #self.state()
            if self.slider_drive_to_limit_switch_action_server.is_preempt_requested():
                rospy.loginfo(rospy.get_caller_id() + " " + self.ns + " recieved cancle request")
                result.success = False
                self.slider.stop()
                return self.slider_drive_to_limit_switch_action_server.set_preempted()
        
        if not self.slider.busy:
            result.success = True
            #self.mutex = True
            
            print(result)
            return self.slider_move_action_server.set_succeeded(result)

    def stop(self,data = None):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", str(data.run) )        
        self.slider.stop()
        return TriggerResponse(success = True, message = self.ns + " stopped slider")
    
    def set_shift(self,data=None):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", str(data.run) )        
        self.slider.set_shift(data.x)
        return set_intResponse(success = True)

    def set_timeout(self,data=None):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", str(data.run) )        
        self.slider.set_timeout(data.x)
        return set_intResponse(success = True)
    
    def state(self,data=None):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", str(data.run) )        
        #self.slider.read()
        #server.slider.pub_busy.publish(server.slider.busy)
        #server.slider.pub_pos.publish(server.slider.pos)
        return TriggerResponse(success = True, message = str(self.ns + " position: " + str(self.slider.pos) + " busy: " + str(self.slider.busy)))

if __name__ == '__main__':
    rospy.init_node('slider_server', anonymous=True)
    ns = ""
    try:
        ns = rospy.get_param('~ns')
    except:
        pass
    server=Slider_Remote_Server(ns)
    rospy.spin()
    '''
    try:
        while not rospy.is_shutdown():
            #server.slider.read()
            server.slider.pub_busy.publish(server.slider.busy)
            server.slider.pub_pos.publish(server.slider.pos)
    except KeyboardInterrupt:
        pass
    '''
