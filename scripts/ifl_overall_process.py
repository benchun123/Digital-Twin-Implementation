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
uarm_move_client = actionlib.SimpleActionClient('my_uArm/move', MoveAction)
uarm_grip_client = actionlib.SimpleActionClient('my_uArm/grip', GraspAction)
uconveyor_move_client = actionlib.SimpleActionClient('my_conveyor/move', MoveAction)

# start node
rospy.init_node('uarm1_client', anonymous=True)


##################################################################
#               methods that call the services                   #
##################################################################


def uarm_move(x, y, z, w) -> MoveAction.action_result:
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
    uarm_move_client.wait_for_server()
    print("connected")

    # Creates a goal to send to the action server.
    goal = MoveGoal(target=[x, y, z, w])
    print("goal defined")

    # Sends the goal to the action server.
    uarm_move_client.send_goal(goal)
    print("goal sent")

    # Waits for the server to finish performing the action.
    uarm_move_client.wait_for_result()

    # or shorter alternative to three commands above:
    # uarm_move_client.send_goal_and_wait(MoveGoal(target=[x, y, z, w]))

    # gets the result of the task
    return uarm_move_client.get_result()


def uram_grip(grab) -> GraspAction.action_result:
    """
    publishes goal for uarm gripper and waits until it is executed
    @param grab:
    @return: GraspAction.action_result
    """

    print("connecting")
    # Waits until the action server has started up and started
    # listening for goals.
    uarm_grip_client.wait_for_server()
    print("connected")

    # Creates a goal to send to the action server.
    goal = GraspGoal(grab=grab)
    print("goal defined")

    # Sends the goal to the action server.
    uarm_grip_client.send_goal(goal)
    print("goal sent")

    # Waits for the server to finish performing the action.
    uarm_grip_client.wait_for_result()

    # gets the result of the task
    return uarm_grip_client.get_result()

def conveyor_move(x, y, z, w) -> MoveAction.action_result:
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
    uconveyor_move_client.wait_for_server()
    print("connected")

    # Creates a goal to send to the action server.
    goal = MoveGoal(target=[x, y, z, w])
    print("goal defined")

    # Sends the goal to the action server.
    uconveyor_move_client.send_goal(goal)
    print("goal sent")

    # Waits for the server to finish performing the action.
    uconveyor_move_client.wait_for_result()

    # or shorter alternative to three commands above:
    # uconveyor_move_client.send_goal_and_wait(MoveGoal(target=[x, y, z, w]))

    # gets the result of the task
    return uconveyor_move_client.get_result()


# conveyor move
print(conveyor_move(-0.4, 0.0, 0.04, 90))
time.sleep(3)
print(conveyor_move(0.1, 0.0, 0.04, 90))
time.sleep(3)

# uarm move
print(uarm_move(0.20, 0.0, 0.04, 90))
print(uram_grip(True))
print(uarm_move(0.20, 0.0, 0.20, 90))
print(uarm_move(0.02, 0.20, 0.16, 90))
print(uram_grip(False))

