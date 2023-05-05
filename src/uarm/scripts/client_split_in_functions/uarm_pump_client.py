#!/usr/bin/env python

from __future__ import print_function
import rospy
from uarm_msgs.srv import *
import actionlib

#call uarm_pump service
def pump(state):
    pump_state = state
    rospy.wait_for_service('uarm_set_pump')
    print("connected")
    try:
        service_object = rospy.ServiceProxy('uarm_set_pump', uarm_pump)
        received_response = service_object(pump_state)
        print("service called")
        return received_response.success
    except:
        print("service call failed")
        return(False)

if __name__ == "__main__":
    rospy.init_node('uarm_pump_client', anonymous=True)
    try:
        while True:
            print("select pump state")
            state = bool(input("True / False : "))
            print(pump(state))
    except:
        pass
