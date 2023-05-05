from typing import Optional
import numpy as np
from omni.isaac.core.robots.robot import Robot
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils.stage import add_reference_to_stage, get_current_stage
import carb
from .slider_controller import SliderController

class SliderModel(Robot):
    """[sunmary]

    """
    def __init__(
        self,
        prim_path: str,
        name: str = "slider_model",
        usd_path: Optional[str] = None,
        position: Optional[np.ndarray] = None,
        scale: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None) -> None:

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
        
        super().__init__(prim_path=prim_path, name=name, position=position, orientation=orientation, scale=scale)
        
        self._stage = get_current_stage()
        self.position = self.get_joint_positions()
        return
    
    def get_joint_pose(self):
        return self.self.get_joint_positions()


    def post_reset(self) -> None:
        """[summary]
        """
        """Executed after reseting the scene
        """
        print("post_reset ") 
        Robot.post_reset(self)

        self.joint_position_np = np.array([0.])
        self.set_joint_positions(self.joint_position_np)
        print("world Positions once: " + str(self.get_world_pose()))

        self.goal_position = self.joint_position_np

        self.slider_articulation_controller = self.get_articulation_controller()
        self.slider_articulation_controller.switch_control_mode(mode="velocity") # default control type
        self.slider_controller = SliderController() # control command
        return

    def enable_move(self, x):
        self.goal_position = [x]

    def update_test(self):
        # print("enable_move start ", self.goal_position)
        position = self.get_joint_positions()
        actions = self.slider_controller.forward(start_position=position,
                                                joint_velocity = 0.1,
                                                goal_position=self.goal_position)

        # perfom actions
        self._articulation_controller.apply_action(actions)
