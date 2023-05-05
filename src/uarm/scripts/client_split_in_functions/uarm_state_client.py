#!/usr/bin/env python

from __future__ import print_function
import rospy
from uarm_msgs.srv import *
import actionlib

#call the uarm_get_state service
def state():
    print("reading uarm state")
    rospy.wait_for_service('uarm_get_state')
    try:
        service_object = rospy.ServiceProxy('uarm_get_state', uarm_get_state)
        received_response = service_object()
        return received_response
    except:
        print("service call failed")
        return(False)

if __name__ == "__main__":
    rospy.init_node('uarm_state_client', anonymous=True)
    try:
        while True:
            input("enter anything for state - nothing to break >>")
            print(state())
    except:
        pass
