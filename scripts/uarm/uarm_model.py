# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from typing import Optional
import numpy as np
from omni.isaac.core.robots.robot import Robot
from omni.isaac.core.prims.rigid_prim import RigidPrim
from omni.isaac.manipulators.grippers.surface_gripper import SurfaceGripper
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
import carb
from .uarm_rmpflow_controller import RMPFlowController
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.core.utils.stage import get_stage_units
 

class UArmModel(Robot): 
    """[summary]

        Args:
            prim_path (str): [description]
            name (str, optional): [description]. Defaults to "ur10_robot".
            usd_path (Optional[str], optional): [description]. Defaults to None.
            position (Optional[np.ndarray], optional): [description]. Defaults to None.
            orientation (Optional[np.ndarray], optional): [description]. Defaults to None.
            end_effector_prim_name (Optional[str], optional): [description]. Defaults to None.
            attach_gripper (bool, optional): [description]. Defaults to False.
            gripper_usd (Optional[str], optional): [description]. Defaults to "default".

        Raises:
            NotImplementedError: [description]
        """

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
        prim = get_prim_at_path(prim_path)
        self._end_effector = None
        self._gripper = None
        self._end_effector_prim_name = end_effector_prim_name
        if not prim.IsValid():
            if usd_path:
                add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
            else:
                assets_root_path = get_assets_root_path()
                if assets_root_path is None:
                    carb.log_error("Could not find Isaac Sim assets folder")
                    return
                usd_path = assets_root_path + "/Isaac/Robots/UR10/ur10.usd"
                add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
            if self._end_effector_prim_name is None:
                self._end_effector_prim_path = prim_path + "/ee_link"
            else:
                self._end_effector_prim_path = prim_path + "/" + end_effector_prim_name
        else:
            # TODO: change this
            if self._end_effector_prim_name is None:
                self._end_effector_prim_path = prim_path + "/ee_link"
            else:
                self._end_effector_prim_path = prim_path + "/" + end_effector_prim_name
        super().__init__(
            prim_path=prim_path, name=name, position=position, orientation=orientation, scale=scale, articulation_controller=None
        )
        self._gripper_usd = gripper_usd
        if attach_gripper:
            if gripper_usd == "default":
                assets_root_path = get_assets_root_path()
                if assets_root_path is None:
                    carb.log_error("Could not find Isaac Sim assets folder")
                    return
                gripper_usd = assets_root_path + "/Isaac/Robots/UR10/Props/short_gripper.usd"
                add_reference_to_stage(usd_path=gripper_usd, prim_path=self._end_effector_prim_path)
                self._gripper = SurfaceGripper(
                    end_effector_prim_path=self._end_effector_prim_path, translate=0.1611, direction="x"
                )
            elif gripper_usd is None:
                carb.log_warn("Not adding a gripper usd, the gripper already exists in the asset")
                self._gripper = SurfaceGripper(
                    end_effector_prim_path=self._end_effector_prim_path, translate=0.1611, direction="x"
                )
            else:
                raise NotImplementedError
        self._attach_gripper = attach_gripper
        return

    @property
    def attach_gripper(self) -> bool:
        """[summary]

        Returns:
            bool: [description]
        """
        return self._attach_gripper

    @property
    def end_effector(self) -> RigidPrim:
        """[summary]

        Returns:
            RigidPrim: [description]
        """
        return self._end_effector

    @property
    def gripper(self) -> SurfaceGripper:
        """[summary]

        Returns:
            SurfaceGripper: [description]
        """
        return self._gripper

    def initialize(self, physics_sim_view=None) -> None:
        """[summary]
        """
        super().initialize(physics_sim_view)
        if self._attach_gripper:
            self._gripper.initialize(physics_sim_view=physics_sim_view, articulation_num_dofs=self.num_dof)
        self._end_effector = RigidPrim(prim_path=self._end_effector_prim_path, name=self.name + "_end_effector")
        self.disable_gravity()
        self._end_effector.initialize(physics_sim_view)
        return

    def post_reset(self) -> None:

        """Executed after reseting the scene
        """
        print("post_reset ") 
        Robot.post_reset(self)
        self._end_effector.post_reset()
        self._gripper.post_reset()

        self.joint_position_np = np.array([0., 0., 0., 0., 0.])
        self.set_joint_positions(self.joint_position_np)
        print("world Positions once: " + str(self.get_world_pose()))
        
        end_effector_position, end_effector_orientation = self.end_effector.get_world_pose()
        self.picking_position = np.array([0.22, 0.0, 0.04]) / get_stage_units()
        self.placing_position = end_effector_position
        self.placing_quat = end_effector_orientation
        
        # test RMPFlowController
        self._uarm_controller = RMPFlowController(
            name="target_follow", 
            robot_articulation=self, 
            attach_gripper=True
        )
        
        # # test PickPlaceController
        # events_dt = [0.01, 0.01, 0.01, 1.0, 0.01, 0.005, 0.005, 1, 0.01, 0.08]
        # events_dt = [0.0001, 0.0001, 0.0001, 0.1, 0.0001, 0.0005, 0.0005, 0.1, 0.001, 0.0008]
        # self._uarm_controller = PickPlaceController(
        #     name="pick_place_controller", 
        #     gripper=self.gripper, 
        #     robot_articulation=self,
        #     events_dt = events_dt
        # )

        self._articulation_controller = self.get_articulation_controller()
        bad_proportional_gains = self._articulation_controller.get_gains()[0]/100
        self._articulation_controller.set_gains(kps = bad_proportional_gains)

        return

    def enable_move(self, placing_position, placing_quat):
        # # test RMPF controller
        self.placing_position = placing_position
        self.placing_quat = placing_quat

    def enable_pump(self, data):
        if data.grab == True:
            self._gripper.forward(action="close")
        else:
            self._gripper.forward(action="open")
        print('data.grab ', data.grab)


    def update_test(self):
        # print("enable_move start ", self.placing_position, self.placing_quat)
        actions = self._uarm_controller.forward(
            target_end_effector_position= self.placing_position,
            target_end_effector_orientation= self.placing_quat
        )

        # perfom actions
        self._articulation_controller.apply_action(actions)

