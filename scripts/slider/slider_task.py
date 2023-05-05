from omni.isaac.core.tasks import BaseTask
from omni.isaac.core.scenes.scene import Scene
from omni.isaac.core.utils.stage import add_reference_to_stage, get_stage_units, get_current_stage
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from .slider_ros import SliderROS
import carb
import numpy as np
from typing import Optional


class SliderTask(BaseTask):
    """Task enable slider    
    """
    # def __init__(self, name: str = "slider") -> None:
    def __init__(self, prim_path: str, name: str = "slider", usd_path: Optional[str] = None, position: Optional[np.ndarray] = None, scale: Optional[np.ndarray] = None, orientation: Optional[np.ndarray] = None) -> None:
        BaseTask.__init__(self, name=name, offset=None)
        self._slider_node = None
        self._prim_path = prim_path
        self._name = name
        self._usd_path = usd_path
        self._position = position
        self._orientation = orientation
        self._scale = scale
        carb.log_info("Loaded slider demo")
        print("Loaded slider demo")
    
    def set_up_scene(self, scene: Scene) -> None:
        """Loads the stage USD and adds the robot and packing bin to the World's scene.

        Args:
            scene (Scene): The world's scene.
        """
        super().set_up_scene(scene)
        self._stage = get_current_stage()
        # self._world.scene.add_default_ground_plane()
        print(" get_stage_units() " ,  get_stage_units()) 
        # base_folder = "/home/benchun/benchun/IsaacProjects/Demonstrator/"
        # # ## load slider
        # asset_path = base_folder + "src/slider/urdf/model/model.usd"
        # add_reference_to_stage(usd_path=asset_path, prim_path="/World/slider")
        # self._slider_node = self.scene.add(
        #     SliderROS(prim_path="/World/slider", 
        #     name="my_slider", 
        #     position=np.array([0, 0, 0]),
        #     scale=np.array([0.01, 0.01, 0.01]),
        # ))

        slider_init_position = self._position / get_stage_units()
        slider_init_orientation = euler_angles_to_quat(self._orientation)
        slider_init_scale = self._scale
        add_reference_to_stage(usd_path=self._usd_path, prim_path=self._prim_path)
        self._slider_node = scene.add(
                SliderROS(prim_path=self._prim_path, 
                            name=self._name, 
                            position=slider_init_position,
                            orientation=slider_init_orientation,
                            scale=slider_init_scale
        ))
        # print("Num of degrees of freedom before first reset: " + str(self._slider_node.num_dof))
        # print("World Positions before first reset: " + str(self._slider_node.get_world_pose()))
        # print("get_stage_units() " + str(get_stage_units()))

        return


