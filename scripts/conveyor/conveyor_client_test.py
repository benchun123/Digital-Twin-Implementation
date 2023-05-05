#!/usr/bin/env python3


import rospy
import actionlib
import time

from robis_messages.msg import MoveAction, MoveGoal

# create ROS action client:
# CLIENT-NAME = actionlib.SimpleActionClient(TOPIC NAME, TYPE OF MESSAGE)
move_client = actionlib.SimpleActionClient('my_conveyor/move', MoveAction)
# move_client = actionlib.SimpleActionClient('conveyor_1/move', MoveAction)

# start node
rospy.init_node('conveyor_client', anonymous=True)


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



#########################################################
#      example for using the defined functions          #
#########################################################
# print(move(0.10, 0.0, 0.04, 90))
# time.sleep(1.5)
# print(move(0.00, 0.0, 0.04, 90))
# time.sleep(1.5)
# print(move(-0.10, 0.0, 0.04, 90))
# time.sleep(1.5)
# print(move(0.00, 0.0, 0.04, 90))

print(move(0.40, 0.0, 0.04, 90))
time.sleep(3)
print(move(0.00, 0.0, 0.04, 90))
time.sleep(3)

