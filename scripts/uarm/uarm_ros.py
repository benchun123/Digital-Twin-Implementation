import numpy as np
import time
import rospy
from typing import Optional, Tuple
from .uarm_model import UArmModel
from std_msgs.msg import Bool
from std_msgs.msg import Float32, Float64
from sensor_msgs.msg import JointState
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.core.utils.stage import get_stage_units
import actionlib
from robis_messages.msg import MoveAction, MoveResult, GraspAction, GraspResult
# from robis_messages.msg import MoveAction, GraspAction, MoveGoal, GraspGoal

class UArmROS(UArmModel):
    def __init__(
        self,
        prim_path: str,
        name: str = "ur10_robot",
        usd_path: Optional[str] = None,
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
        scale: Optional[np.ndarray] = None,
        end_effector_prim_name: Optional[str] = None,
        attach_gripper: bool = False,
        gripper_usd: Optional[str] = "default",
    ) -> None:
        super().__init__(prim_path, name, usd_path, position, orientation, scale, end_effector_prim_name, attach_gripper, gripper_usd)


        self.mutex = True
        self.move_action_server = actionlib.SimpleActionServer(self.name+"/move", MoveAction, self.move, auto_start = False)
        self.move_action_server.start()
        self.grip_action_server = actionlib.SimpleActionServer(self.name+"/grip", GraspAction, self.set_pump, auto_start = False)
        self.grip_action_server.start()

        # # initialize Subscribers
        # self.lis = rospy.Subscriber(self.name+"/joint_states", JointState, self.__joint_state_callback)

        # # initialize publishers
        # self.pub1 = rospy.Publisher(self.name+"/joint1/command", Float64, queue_size=1)
        # self.pub2 = rospy.Publisher(self.name+"/joint2/command", Float64, queue_size=1)
        # self.pub3 = rospy.Publisher(self.name+"/joint3/command", Float64, queue_size=1)
        # self.pub8 = rospy.Publisher(self.name+"/joint4/command", Float64, queue_size=1)
        # self.pubtcp = rospy.Publisher(self.name+"/jointtcp/command", Float64, queue_size=1)


    def move(self, data):
        print("receive action client request,", data, type(data))
        x, y, z, w = data.target
        result = MoveResult()
        if self.mutex == True:
            self.mutex = False

            placing_position = np.array([x, y, z]) / get_stage_units()
            placing_quat = euler_angles_to_quat(np.array([0, 0, 0]))
            self.enable_move(placing_position, placing_quat)

            reached = False
            for check_id in range(3): # check every 2s, check 5 times, maximum 10s
                print('check_id ', check_id)
                reached = self.check_reach_goal(x, y, z)
                if reached == True:
                    break
                else:
                    time.sleep(2)

            result.success = reached
            # result.success = True
            self.mutex = True
            return self.move_action_server.set_succeeded(result)
        else:
            rospy.loginfo('%s: working on other task - target rejected' % (str(self.name+'uarm_move server')))
            result.success = False
            return self.move_action_server.set_succeeded(result)
    
    def set_pump(self, data):
        print('set pump ', data)
        result = GraspResult()
        self.mutex = True
        if self.mutex == True:
            self.mutex = False

            self.enable_pump(data)
            
            result.success = (self.gripper.is_closed() == data.grab)
            # result.success = True
            print('result.success ', result.success)
            self.mutex = True
            return self.grip_action_server.set_succeeded(result)
        else:
            result.success = False
            return self.grip_action_server.set_succeeded(result)

    def check_reach_goal(self, x, y, z):
        thres = 0.005
        end_effector_position, end_effector_orientation = self.end_effector.get_world_pose()
        print('target ', x, y, z)
        print('current ', end_effector_position)
        x_cur, y_cur, z_cur =  end_effector_position
        if x-x_cur <= thres and x-x_cur >= -thres and \
            y-y_cur <= thres and y-y_cur >= -thres and \
            z-z_cur <= thres and z-z_cur >= -thres :
            return True
        else:
            return False

    def check_punp_state(self):
        return self.gripper.is_closed() 

    def check_arm_state(self):
        end_effector_position, end_effector_orientation = self.end_effector.get_world_pose()
        return end_effector_position
