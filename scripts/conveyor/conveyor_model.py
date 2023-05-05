from typing import Optional
import numpy as np
import time
from omni.isaac.core.robots.robot import Robot
from omni.isaac.core.prims import XFormPrim, RigidPrim
# from omni.isaac.core.utils.nucleus import get_assets_root_path, get_server_path
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils.stage import add_reference_to_stage, get_current_stage
import omni.graph.core as og
from pxr import UsdGeom, Gf, UsdPhysics, PhysxSchema
from omni.isaac.conveyor.scripts.commands import CreateConveyorBelt
import carb
from .conveyor_controller import ConveryorController

class ConveyorModel(XFormPrim):
    """[sunmary]

    """
    def __init__(
        self,
        prim_path: str,
        name: str = "conveyor_model",
        usd_path: Optional[str] = None,
        position: Optional[np.ndarray] = None,
        scale: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None) -> None:

        self._prim_path = prim_path
        print("self._prim_path", self._prim_path)
        prim = get_prim_at_path(prim_path)
        if not prim.IsValid():
            if usd_path:
                add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
            else:
                # assets_root_path = get_server_path()
                # if assets_root_path is None:
                #     carb.log_error("Could not find Isaac Sim assets folder")
                #     return
                # usd_path = assets_root_path + "/Projects/AgiProbot/TransferUnit/TransferUnit.usd"
                usd_path = "/Projects/AgiProbot/TransferUnit/TransferUnit.usd"
                add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
        super().__init__(prim_path=prim_path, name=name, position=position, orientation=orientation, scale=scale)
        
        self._stage = get_current_stage()
        self.conveyor_core = self._stage.GetPrimAtPath(self._prim_path + "/Plane/ConveyorBeltGraph/ConveyorNode")
        self.conveyor_velocity_command = self.conveyor_core.GetAttribute("inputs:velocity")
        self.conveyor_velocity_command.Set(0.)
        self.conveyor_current_velocity = UsdPhysics.RigidBodyAPI(self._stage.GetPrimAtPath(self._prim_path + "/Plane")).GetVelocityAttr()
        
        # simulate joint movement
        self.joint_pos_sim = [0.]
        self.timestamp_sim = 0.
        self.velocity_sim = 0.
        self.goal_position = [0.]
        
        return
    
    def get_left_conveyor_velocity(self):
        return self.conveyor_current_velocity.Get()[0]

    def set_left_conveyor_velocity(self, velocity: float):
        try:
            self.conveyor_velocity_command.Set(velocity)
            return True
        except:
            return False

    def post_reset(self) -> None:
        """[summary]
        """
        """Executed after reseting the scene
        """
        print("post_reset ") 
        # Robot.post_reset(self)

        self.joint_pos_sim = [0.]
        self.timestamp_sim = 0.
        self.velocity_sim = 0.
        self.goal_position = [0.]
        # self.slider_articulation_controller = self.get_articulation_controller()
        # self.slider_articulation_controller.switch_control_mode(mode="velocity") # default control type
        self.conveyor_controller = ConveryorController() # control command
        return

    def enable_move(self, x):
        # print("self.timestamp_sim ", self.timestamp_sim)
        self.timestamp_sim = time.time()
        self.goal_position = [x]

    def update_test(self):
        # print("enable_move start ", self.goal_position)
        curr_position = self.calculate_joint_positions()
        velocity_cmd = self.conveyor_controller.forward(start_position=curr_position,
                                                joint_velocity = 0.1,
                                                goal_position=self.goal_position)
        self.velocity_sim = velocity_cmd[0]
        # print("goal_position ", self.goal_position)
        # print("curr_position ", curr_position)
        # print("velocity_cmd ", velocity_cmd)
        try:
            self.conveyor_velocity_command.Set(velocity_cmd[0])
            return True
        except:
            return False

    def calculate_joint_positions(self):
        # print("time_delta ", (time.time() - self.timestamp_sim))
        time_delta = (time.time() - self.timestamp_sim)*2 ## why *2, test value ....
        self.timestamp_sim = time.time() 
        curr_pos_tmp = self.joint_pos_sim[0] + time_delta*self.velocity_sim
        self.joint_pos_sim = [curr_pos_tmp]
        return self.joint_pos_sim 

