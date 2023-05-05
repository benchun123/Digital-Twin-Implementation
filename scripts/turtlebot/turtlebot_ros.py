import numpy as np
import time
import rospy
from typing import Optional
from .turtlebot_model import TurtlebotModel
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import actionlib
from robis_messages.msg import MoveAction, MoveResult
# need to change to mbf_msgs.MoveBaseGoal

class TurtlebotROS(TurtlebotModel):
    def __init__(
        self, 
        prim_path: str, 
        name: str = "turtle_model", 
        usd_path: Optional[str] = None, 
        position: Optional[np.ndarray] = None, 
        scale: Optional[np.ndarray] = None, 
        orientation: Optional[np.ndarray] = None, 
        wheel_dof_names: Optional[str] = None ) -> None:
        super().__init__(prim_path, name, usd_path, position, scale, orientation, wheel_dof_names)

        self.mutex = True
        # self.move_action_server = actionlib.SimpleActionServer(self.name+"/move", MoveAction, self.move, auto_start = False)
        # self.move_action_server.start()
        self.sub_cmd_topic = rospy.Subscriber(self.name + "/cmd_vel", Twist, self.sub_vel_cmd)
        self.pub_joint_pos = rospy.Publisher(self.name + "/state", Float32, queue_size=1)

    def move(self, data):
        print("receive action client request,", data, type(data))
        x, y, z, w = data.target
        result = MoveResult()
        if self.mutex == True:
            self.mutex = False

            self.enable_move(x, y, z)
            reached = False
            for check_id in range(5): # check every 2s, check 5 times, maximum 10s
                print('check_id ', check_id)
                reached = self.check_reach_goal(x, y, z)
                if reached == True:
                    break
                else:
                    time.sleep(3)

            result.success = reached
            self.mutex = True
            return self.move_action_server.set_succeeded(result)
        else:
            rospy.loginfo('%s: working on other task - target rejected' % (str(self.name+'uarm_move server')))
            result.success = False
            return self.move_action_server.set_succeeded(result)
    
    def check_reach_goal(self, x, y, z):
        thres = 0.01
        current_pos, orientation = self.get_world_pose()
        print('target ', x, y, z)
        print('current ', current_pos)
        x_cur, y_cur, z_cur = current_pos
        if x-x_cur <= thres and x-x_cur >= -thres and \
            y-y_cur <= thres and y-y_cur >= -thres:
            return True
        else:
            return False


    def sub_vel_cmd(self, msg):
        rospy.loginfo("Received a /cmd_vel message!")
        rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
        rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))
        print("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
        print("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))
        self.vel_cmd = [msg.linear.x, msg.angular.z]

    def publish_all(self):
        self.pub_joint_pos.publish(self.get_joint_positions())
