#!/usr/bin/env python3


import rospy
import actionlib
import time

from robis_messages.msg import MoveAction, GraspAction, MoveGoal, GraspGoal

# call uarm_move service

#################################
#                               #
#       valid positions         #
#                               #
#      x element [0, 280]       #
#                               #
#      y element [0, 280]       #
#                               #
#      z element [-100, 150]    #
#                               #
#      w element [0, 180]       #
#                               #
#  RADIUS in x-y Ebene > 150    #
#                               #
#################################


# create ROS action client:
# CLIENT-NAME = actionlib.SimpleActionClient(TOPIC NAME, TYPE OF MESSAGE)
move_client = actionlib.SimpleActionClient('my_uArm/move', MoveAction)
grip_client = actionlib.SimpleActionClient('my_uArm/grip', GraspAction)

# start node
rospy.init_node('uarm1_client', anonymous=True)


##################################################################
#               methods that call the services                   #
##################################################################


def move(x, y, z, w) -> MoveAction.action_result:
    """
    publishes goal for uarm and waits until it is reached
    @param x:
    @param y:
    @param z:
    @param w:
    @return: MoveAction.action_result
    """

    print("connecting")
    # Waits until the action server has started up and started
    # listening for goals.        
    move_client.wait_for_server()
    print("connected")

    # Creates a goal to send to the action server.
    goal = MoveGoal(target=[x, y, z, w])
    print("goal defined")

    # Sends the goal to the action server.
    move_client.send_goal(goal)
    print("goal sent")

    # Waits for the server to finish performing the action.
    move_client.wait_for_result()

    # or shorter alternative to three commands above:
    # move_client.send_goal_and_wait(MoveGoal(target=[x, y, z, w]))

    # gets the result of the task
    return move_client.get_result()


def grip(grab) -> GraspAction.action_result:
    """
    publishes goal for uarm gripper and waits until it is executed
    @param grab:
    @return: GraspAction.action_result
    """

    print("connecting")
    # Waits until the action server has started up and started
    # listening for goals.
    grip_client.wait_for_server()
    print("connected")

    # Creates a goal to send to the action server.
    goal = GraspGoal(grab=grab)
    print("goal defined")

    # Sends the goal to the action server.
    grip_client.send_goal(goal)
    print("goal sent")

    # Waits for the server to finish performing the action.
    grip_client.wait_for_result()

    # gets the result of the task
    return grip_client.get_result()


#########################################################
#      example for using the defined functions          #
#########################################################
print(move(0.20, 0.0, 0.04, 90))
print(grip(True))
print(move(0.20, 0.0, 0.16, 90))
print(move(0.0, 0.20, 0.16, 90))
print(move(0.0, 0.20, 0.04, 90))
print(grip(False))
print(move(0.0, 0.20, 0.16, 90))
print(move(0.0, 0.20, 0.04, 90))
print(grip(True))
print(move(0.0, 0.20, 0.16, 90))
print(move(0.20, 0.0, 0.16, 90))
print(grip(False))

# print(move(0.20, 0.0, 0.04, 90))
# time.sleep(5)
# print(grip(True))
# print(move(0.20, 0.0, 0.16, 90))
# time.sleep(5)
# print(move(0.0, 0.20, 0.16, 90))
# time.sleep(5)
# print(move(0.0, 0.20, 0.04, 90))
# time.sleep(5)
# print(grip(False))
# print(move(0.0, 0.20, 0.16, 90))
# time.sleep(5)
# print(move(0.0, 0.20, 0.04, 90))
# time.sleep(5)
# print(grip(True))
# print(move(0.0, 0.20, 0.16, 90))
# time.sleep(5)
# print(move(0.20, 0.0, 0.16, 90))
# time.sleep(5)
# print(grip(False))
# # call move function and print result
# print(move(180, 180, 33, 90))

# call pump function and print result

# time.sleep(2)
# # call move function and print result
# # print(move(180, 0, 130, 90))

# # call move function and print result
# print(move(180, 180, 33, 90))

# time.sleep(1)

# # call punp function and print result
# print(grip(False))

# # call move function and print result
# print(move(180, 0, 130, 90))
