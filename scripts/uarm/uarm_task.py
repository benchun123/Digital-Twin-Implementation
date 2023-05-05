from omni.isaac.core.tasks import BaseTask
from omni.isaac.core.scenes.scene import Scene
from omni.isaac.core.utils.stage import add_reference_to_stage, get_stage_units, get_current_stage
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from .uarm_ros import UArmROS
import carb
import numpy as np
from typing import Optional


class UarmTask(BaseTask):
    """Task enable uarm    
    """
    # def __init__(self, name: str = "uarm") -> None:
    def __init__(self, prim_path: str, name: str = "uarm", usd_path: Optional[str] = None, position: Optional[np.ndarray] = None, scale: Optional[np.ndarray] = None, orientation: Optional[np.ndarray] = None) -> None:
        BaseTask.__init__(self, name=name, offset=None)
        self._uarm_node = None
        self._prim_path = prim_path
        self._name = name
        self._usd_path = usd_path
        self._position = position
        self._orientation = orientation
        self._scale = scale
        carb.log_info("Loaded uarm demo")
        print("Loaded uarm demo")
    
    def set_up_scene(self, scene: Scene) -> None:
        """Loads the stage USD and adds the robot and packing bin to the World's scene.

        Args:
            scene (Scene): The world's scene.
        """
        super().set_up_scene(scene)
        self._stage = get_current_stage()
        print(" get_stage_units() " ,  get_stage_units()) 

        # base_folder = "/home/benchun/benchun/IsaacProjects/Demonstrator/"
        # # ## load uarm
        # asset_path = base_folder + "src/uarm/urdf/uArm_light_model/uArm_light_model.usd"
        # add_reference_to_stage(usd_path=asset_path, prim_path="/World/uarm")
        # self._uarm_node = self.scene.add(
        #     UArmROS(prim_path="/World/uarm", 
        #     name="my_uArm", 
        #     position=np.array([0, 0, 0]),
        #     # scale=np.array([0.01, 0.01, 0.01]),
        #     end_effector_prim_name = "left_end_effector",
        #     gripper_usd=None, attach_gripper=True)
        # )

        uarm_init_position = self._position / get_stage_units()
        uarm_init_orientation = euler_angles_to_quat(self._orientation)
        uarm_init_scale = self._scale
        add_reference_to_stage(usd_path=self._usd_path, prim_path=self._prim_path)
        self._uarm_node = scene.add(
                UArmROS(prim_path=self._prim_path, 
                            name=self._name, 
                            position=uarm_init_position,
                            orientation=uarm_init_orientation,
                            scale=uarm_init_scale,
                            end_effector_prim_name = "left_end_effector",
                            gripper_usd=None, 
                            attach_gripper=True
        ))
        self._uarm_node.gripper.set_translate(value=0) # important
        self._uarm_node.gripper.set_direction(value="x")

        print("Num of degrees of freedom before first reset: " + str(self._uarm_node.num_dof))
        print("World Positions before first reset: " + str(self._uarm_node.get_world_pose()))
        print("get_stage_units() " + str(get_stage_units()))

        return


