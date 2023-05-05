import numpy as np
import time
import rospy
from typing import Optional
from .slider_model import SliderModel
from std_msgs.msg import Bool
from std_msgs.msg import Float32
import actionlib
from robis_messages.msg import MoveAction, MoveResult

class SliderROS(SliderModel):
    def __init__(self, prim_path: str, name: str = "slider_model", usd_path: Optional[str] = None, position: Optional[np.ndarray] = None, scale: Optional[np.ndarray] = None, orientation: Optional[np.ndarray] = None) -> None:
        super().__init__(prim_path, name, usd_path, position, scale, orientation)

        self.mutex = True
        self.move_action_server = actionlib.SimpleActionServer(self.name+"/move", MoveAction, self.move, auto_start = False)
        self.move_action_server.start()
        # self._setLeftSpeed_service = rospy.Service(self.name + "/set_left_speed", SetSpeed, self.set_left_speed)
        self.pub_joint_pos = rospy.Publisher(self.name + "/state", Float32, queue_size=1)

    def move(self, data):
        print("receive action client request,", data, type(data))
        x, y, z, w = data.target
        result = MoveResult()
        if self.mutex == True:
            self.mutex = False

            self.enable_move(x)
            reached = False
            for check_id in range(3): # check every 2s, check 5 times, maximum 10s
                print('check_id ', check_id)
                reached = self.check_reach_goal(x, y, z)
                if reached == True:
                    break
                else:
                    time.sleep(1)

            result.success = True
            self.mutex = True
            return self.move_action_server.set_succeeded(result)
        else:
            rospy.loginfo('%s: working on other task - target rejected' % (str(self.name+'uarm_move server')))
            result.success = False
            return self.move_action_server.set_succeeded(result)
    

    # def set_left_speed(self, req):
    #     rospy.loginfo("Received a speed request for the left conveyor: %f", req.speed)
    #     return SetSpeedResponse(self.set_left_conveyor_velocity(req.speed))

    def check_reach_goal(self, x, y, z):
        thres = 0.005
        current_pos = self.get_joint_positions()
        print('target ', x, y, z)
        print('current ', current_pos)
        x_cur = current_pos[0]
        if x-x_cur <= thres and x-x_cur >= -thres:
            return True
        else:
            return False


    def publish_all(self):
        self.pub_joint_pos.publish(self.get_joint_positions())
