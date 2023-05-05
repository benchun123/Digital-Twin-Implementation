#!/usr/bin/env python
import rospy
import actionlib
from uarm_msgs.srv import *
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Bool
import uarm_msgs.msg
import time
import json
import requests
import threading
from isaac_interface import uArm_Isaac_Interface

#################################
#                               #
#       valid positions         #
#                               #
#      x element [30, 280]      #
#                               #
#      y element [-280, 280]    #
#                               #
#      z element [0, 130]       #
#                               #
#  RADIUS in x-y Ebene > 150    #
#                               #
#################################

class uArm_Isaac_Server():

    def __init__(self,ns):

        self.busy = False
        self.ns = ns
        self.name = ns
        self.success = False

        self.move_action_server = actionlib.SimpleActionServer(str(self.name+'uarm_move'), uarm_msgs.msg.uarm_moveAction, self.move, auto_start = False)
        self.move_action_server.start()
        self.set_pump_service = rospy.Service(str(self.name+'uarm_set_pump'), uarm_pump, self.set_pump)
        self.recalibration_action_server = actionlib.SimpleActionServer(str(self.name+'uarm_reset'), uarm_msgs.msg.uarm_moveAction, self.reset, auto_start = False)
        self.recalibration_action_server.start()
        self.state_service = rospy.Service(str(self.name+'uarm_get_state'), uarm_get_state, self.get_state)
        self.stop_service = rospy.Service(str(self.name+'stop'), Trigger, self.stop_uArm)
        self.auto_move_action_server = actionlib.SimpleActionServer(str(self.name+'uarm_auto_move'), uarm_msgs.msg.uarm_moveAction, self.auto_move, auto_start = False)
        self.auto_move_action_server.start()

        # move to uArm_Isaac_Interface
        # self.pub_busy = rospy.Publisher(str(self.name+'busy'), Bool, queue_size=1)
        # self.publisher = threading.Thread(target=self.pub_state)
        # self.publisher.start()
        
        self.uarm = uArm_Isaac_Interface(ns = self.ns)
        self.uarm.run_simulation()

        rospy.loginfo('%s: started' % (str(self.name+'server')))

    def check_limits(self,x,y,z,w):
        lim_x = [30,280]
        lim_y = [-278,278]
        lim_z = [-120, 130]
        lim_r = [150,280]
        lim_w = [0,180]
        limits = [lim_x,lim_y,lim_z,lim_w,lim_r]
        r = (x**2+y**2)**0.5
        coordinates = [x,y,z,w,r]
        for i in [0,1,2,3]:
            if coordinates[i] < limits[i][0] or coordinates[i] > limits[i][1]:
                print("invalid target")
                return False
        return True

    def move(self, data):
        result = uarm_msgs.msg.uarm_moveResult()
        if not self.busy:
            # api-endpoint 
            if not self.check_limits(int(data.x),int(data.y),int(data.z),int(data.w)):
                rospy.loginfo(rospy.get_caller_id() + " " + self.ns + " target rejected")
                result.success = False
                return self.move_action_server.set_succeeded(result)
            
            self.uarm.move(int(data.x),int(data.y),int(data.z),int(data.w))
            rospy.loginfo(rospy.get_caller_id() + " " + self.ns + " target accepted")

            while self.busy:
                if self.move_action_server.is_preempt_requested():
                    print("recieved cancle request")
                    result.success = False
                    #self.uarm.stop()
                    return self.move_action_server.set_preempted()
            
            
            rospy.loginfo('%s: new goal reached [%i, %i, %i, %i]' % (str(self.name), int(data.x),int(data.y),int(data.z),int(data.w)))

            result.success = self.success
            self.success = False
            return self.move_action_server.set_succeeded(result)
        else:
            rospy.loginfo('%s: working on other task - target rejected' % (str(self.name+'uarm_move server')))
            result.success = False
            return self.move_action_server.set_succeeded(result)

    #pick up cargo and delivers to destination
    def auto_move(self, data):
        result = uarm_msgs.msg.uarm_moveResult()
        if not self.busy:

            if not self.check_limits(int(data.x),int(data.y),int(data.z),int(data.w)):
                result.success = False
                return self.auto_move_action_server.set_succeeded(result)

            # api-endpoint 
            self.uarm.auto_move(int(data.x),int(data.y),int(data.z),int(data.w))
            rospy.loginfo(rospy.get_caller_id() + " " + self.ns + " target accepted")

            while self.busy:
                if self.auto_move_action_server.is_preempt_requested():
                    print("recieved cancle request")
                    result.success = False
                    #self.uarm.stop()
                    return self.auto_move_action_server.set_preempted()

            result.success = self.success
            rospy.loginfo('%s: new goal reached [%i, %i, %i, %i]' % (str(self.name), int(data.x),int(data.y),int(data.z),int(data.w)))
            return self.auto_move_action_server.set_succeeded(result)
        else:
            result.success = self.success
            return self.auto_move_action_server.set_succeeded(result)

    def set_pump(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", str(data.run) )
        if not self.busy:
            # self.send_pump(data.run)
            self.uarm.pump(data.run)
            return uarm_pumpResponse(True)
        else:
            return uarm_pumpResponse(False)

    def reset(self, data=None):
        result = uarm_msgs.msg.uarm_moveResult()
        if not self.busy:
            # self.send_reset()
            self.uarm.reset()
            while self.busy:
                if self.recalibration_action_server.is_preempt_requested():
                    print("recieved cancle request")
                    result.success = False
                    #self.uarm.stop()
                    return self.recalibration_action_server.set_preempted()
            
            rospy.loginfo('%s: resetted' % (str(self.name)))
            result.success = self.success
            return self.recalibration_action_server.set_succeeded(result)
        else:
            result.success = self.success
            return self.recalibration_action_server.set_succeeded(result)

    #service instead of publisher, because real uArm has to be stopped to get position
    def get_state(self, data=None):
        rospy.loginfo(rospy.get_caller_id() + " state requested" )
        if not self.busy:
            pump = self.uarm.pump_state
            world_pos = self.uarm.robot.get_world_pose()
            x = int(world_pos[0][0])
            y = int(world_pos[0][1])
            z = int(world_pos[0][2])
            return uarm_get_stateResponse(ready = True, pump = pump, x = x, y = y, z = z)
        else:
            return uarm_get_stateResponse(ready = False, pump = pump, x = 0, y = 0, z = 0)

    # TODO: How to stop uArm
    def stop_uArm(self, data=None):
        rospy.loginfo(rospy.get_caller_id() + " stopping" )
        # try:
        #     self.busy = True
        #     # api-endpoint 
        #     URL = "http://127.0.0.1:5000/move" 
        #     PARAMS = {'task': "stop"}
        #     # sending get request and saving the response as response object 
        #     r = requests.get(url = URL, json = PARAMS)
        #     self.success = bool(r.json()['success'])
        #     #print(self.success)
        #     self.busy = False
        # except:
        #     pass
        
        if self.success:            
            return TriggerResponse(success = True, message = str(self.name + " stopped uArm"))
        else:
            return TriggerResponse(success = False, message = str(self.name + " stopping uArm failed"))

    # def pub_state(self):
    #     r = rospy.Rate(1)
    #     while not rospy.is_shutdown():

    #         self.pub_busy.publish(self.busy)
    #         #self.pub_state.publish(self.state)
    #         r.sleep()

if __name__ == '__main__':
    rospy.init_node('uarm_server', anonymous=True)

    ns = "uarm1/"
    #ns = ""
    try:
        ns = rospy.get_param('~ns')
    except:
        pass
    server = uArm_Isaac_Server(ns)
    rospy.spin()
