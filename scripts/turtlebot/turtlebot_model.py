from typing import Optional
import numpy as np
import time
import carb
# from omni.isaac.core.robots.robot import Robot
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils.stage import add_reference_to_stage, get_current_stage

from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
# from omni.isaac.motion_generation import WheelBasePoseController

class TurtlebotModel(WheeledRobot):
    """[sunmary]

    """
    def __init__(
        self,
        prim_path: str,
        name: str = "turtlebot_model",
        usd_path: Optional[str] = None,
        position: Optional[np.ndarray] = None,
        scale: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
        wheel_dof_names: Optional[str] = None) -> None:

        self._prim_path = prim_path
        prim = get_prim_at_path(prim_path)
        if not prim.IsValid():
            if usd_path:
                add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
            else:
                carb.log_error("Could not find Isaac Sim assets folder")
                return
                # usd_path = "/Projects/AgiProbot/TransferUnit/TransferUnit.usd"
                # add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
        
        super().__init__(prim_path=prim_path, name=name, position=position, orientation=orientation, scale=scale, wheel_dof_names=wheel_dof_names)
        self._stage = get_current_stage()
        self.position = self.get_joint_positions()
        self.vel_cmd = [0.0, 0.0]
        return
    
    def get_joint_pose(self):
        return self.self.get_joint_positions()

    def post_reset(self) -> None:
        """[summary]
        """
        """Executed after reseting the scene
        """
        print("post_reset ") 
        # self.joint_position_np = np.array([0., 0., 0.,])
        # self.set_joint_positions(self.joint_position_np)
        # print("world Positions once: " + str(self.get_world_pose()))

        # self.goal_position = np.array([0., 0., 0.,])

        # self.turtlebot_controller = WheelBasePoseController(name="cool_controller",
        #                                                     open_loop_wheel_controller=
        #                                                     DifferentialController(name="simple_control",
        #                                                                             wheel_radius=0.025, wheel_base=0.16),
        #                                                     is_holonomic=False)
        self.turtlebot_controller = DifferentialController(name="simple_control",
                                    wheel_radius=0.025, wheel_base=0.16)
        return

    def enable_move(self, x, y, z):
        # self.goal_position = [x, y, z]
        self.goal_position=np.array([x, y, z])

    def update_test(self):
        # print("enable_move start ", self.goal_position)
        # position, orientation = self.get_world_pose()
        # actions = self.turtlebot_controller.forward(start_position=position,
        #                                             start_orientation=orientation,
        #                                             lateral_velocity = 0.2,
        #                                             yaw_velocity = 2, # degree
        #                                             heading_tol = 0.03,
        #                                             position_tol = 0.04,
        #                                             goal_position= self.goal_position,
        #                                             # goal_orientation = euler_angles_to_quat(np.array([0, 0, 1.57]))
        #                                             )
        # self.vel_cmd = [0.1, 10.0/90.0*1.57]
        actions = self.turtlebot_controller.forward(command=self.vel_cmd)
        # perfom actions
        self.apply_action(actions)
