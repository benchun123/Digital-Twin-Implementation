#!/usr/bin/env python


import rospy
import uarm_msgs.msg
from uarm_msgs.srv import *
from std_srvs.srv import *
import actionlib
import time



#call uarm_move service

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


        
#create ROS action client:
#CLIENT-NAME = actionlib.SimpleActionClient(TOPIC NAME, TYPE OF MESSAGE)
client = actionlib.SimpleActionClient('uarm1/uarm_move', uarm_msgs.msg.uarm_moveAction)


#start node
rospy.init_node('uarm1_client', anonymous=True)

##################################################################
#               methods that call the services                   #
##################################################################

#publishes goal for uarm and waits until it is reached

def move_client(x, y, z, w):
    
    print("connecting")
    # Waits until the action server has started up and started
    # listening for goals.        
    client.wait_for_server()
    print("connected")
        
    # Creates a goal to send to the action server.
    goal = uarm_msgs.msg.uarm_moveGoal(x=x,y=y,z=z, w=w)
    print("goal defined")
        
    # Sends the goal to the action server.
    client.send_goal(goal)
    print("goal sent")
        
    # Waits for the server to finish performing the action.
    client.wait_for_result()
        
    #gets the result of the task
    return client.get_result()




#publishes service request to change the state of the pump of the vacuum gripper
def pump(pump_state):            
    rospy.wait_for_service('uarm1/uarm_set_pump')
    print("connected")
    try:
        service_object = rospy.ServiceProxy('uarm1/uarm_set_pump', uarm_pump)
        received_response = service_object(pump_state)
        print("service called")
        return received_response.success
    except:
        print("service call failed")
        return(False)









#########################################################
#      example for using the defined functions          #
#########################################################

print('''
uarm simple demonstration:
1. picking up box at (180,180,30)
2. pick up box and place it there again

!!!

wait for the following message before you start the demo:

* Running on http://127.0.0.1:5000/ (Press CTRL+C to quit)

!!!

''')

try:
    input("Enter to move above box pickup position")

except:
    pass
    
#call move function and print result
print(move_client(180,180,33,0))

try:
    input("Enter to start demo\n")
except:
    pass

print(move_client(180,180,27,0))

time.sleep(1)

#call punp function and print result
print(pump(True))

#call move function and print result
print(move_client(180,0,130,90))

time.sleep(1)

#call move function and print result
print(move_client(180,180,30,180))

time.sleep(1)

#call punp function and print result
print(pump(False))

time.sleep(1)

#call move function and print result
print(move_client(180,0,130,90))

