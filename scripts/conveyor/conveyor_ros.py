import numpy as np
import rospy
import time
from typing import Optional
from .conveyor_model import ConveyorModel
# from transfer_msgs.srv import SetSpeed, SetSpeedResponse
from std_msgs.msg import Bool
from std_msgs.msg import Float32
import actionlib
from robis_messages.msg import MoveAction, MoveResult


class ConveyorROS(ConveyorModel):
    def __init__(self, prim_path: str, name: str = "conveyor_model", usd_path: Optional[str] = None, position: Optional[np.ndarray] = None, scale: Optional[np.ndarray] = None, orientation: Optional[np.ndarray] = None) -> None:
        super().__init__(prim_path, name, usd_path, position, scale, orientation)

        self.mutex = True
        self.move_action_server = actionlib.SimpleActionServer(self.name+"/move", MoveAction, self.move, auto_start = False)
        self.move_action_server.start()
        # self._setLeftSpeed_service = rospy.Service(self.name + "/set_left_speed", SetSpeed, self.set_left_speed)
        # self._speed_left_publisher = rospy.Publisher(self.name + "/speed_left", Float32, queue_size=1)
        self.pub_joint_pos = rospy.Publisher(self.name + "/state", Float32, queue_size=1)

    def move(self, data):
        print("receive action client request,", data, type(data))
        x, y, z, w = data.target
        result = MoveResult()
        if self.mutex == True:
            self.mutex = False

            # goal_speed = x
            self.enable_move(x)
            reached = False
            for check_id in range(4): # check every 2s, check 5 times, maximum 10s
                print('check_id ', check_id)
                reached = self.check_reach_goal(x, y, z)
                if reached == True:
                    break
                else:
                    time.sleep(1)

            result.success = reached
            self.mutex = True
            return self.move_action_server.set_succeeded(result)
        else:
            rospy.loginfo('%s: working on other task - target rejected' % (str(self.name+'uarm_move server')))
            result.success = False
            return self.move_action_server.set_succeeded(result)
    

    # def set_left_speed(self, req):
    #     rospy.loginfo("Received a speed request for the left conveyor: %f", req.speed)
    #     return SetSpeedResponse(self.set_left_conveyor_velocity(req.speed))
    # def publish_all(self):
    #     self._speed_left_publisher.publish(self.get_left_conveyor_velocity())

    def publish_all(self):
        self.pub_joint_pos.publish(self.joint_pos_sim[0])

    def check_reach_goal(self, x, y, z):
        thres = 0.005
        current_pos = self.joint_pos_sim
        print('target ', x, y, z)
        print('current ', current_pos)
        x_cur = current_pos[0]
        if x-x_cur <= thres and x-x_cur >= -thres:
            return True
        else:
            return False

