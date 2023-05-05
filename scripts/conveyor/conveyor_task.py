from omni.isaac.core.tasks import BaseTask
from omni.isaac.core.scenes.scene import Scene
from omni.isaac.core.utils.stage import add_reference_to_stage, get_stage_units, get_current_stage
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from .conveyor_ros import ConveyorROS
import carb
import numpy as np
from typing import Optional

class ConveyorTask(BaseTask):
    """Task enable conveyor    
    """
    def __init__(self, prim_path: str, name: str = "conveyor", usd_path: Optional[str] = None, position: Optional[np.ndarray] = None, scale: Optional[np.ndarray] = None, orientation: Optional[np.ndarray] = None) -> None:
        BaseTask.__init__(self, name=name, offset=None)
        self._conveyor_node = None
        self._prim_path = prim_path
        self._name = name
        self._usd_path = usd_path
        self._position = position
        self._orientation = orientation
        self._scale = scale
        
        # self._assets_root_path = get_assets_root_path()
        # if self._assets_root_path is None:
        #     carb.log_error("Could not find Isaac Sim assets folder")
        #     return
        carb.log_info("Loaded conveyor demo")
        print("Loaded conveyor demo")
    
    def set_up_scene(self, scene: Scene) -> None:
        """Loads the stage USD and adds the robot and packing bin to the World's scene.

        Args:
            scene (Scene): The world's scene.
        """
        super().set_up_scene(scene)
        self._stage = get_current_stage()
        # self._world.scene.add_default_ground_plane()
        # print(" get_stage_units() " ,  get_stage_units()) 

        # ## load conveyor
        # base_folder = "/home/benchun/benchun/IsaacProjects/Demonstrator/"
        # asset_path = base_folder + "src/conveyor/mesh/conveyor.usd"
        conveyor_init_position = self._position / get_stage_units()
        conveyor_init_orientation = euler_angles_to_quat(self._orientation)
        conveyor_init_scale = self._scale
        add_reference_to_stage(usd_path=self._usd_path, prim_path=self._prim_path)
        self._conveyor_node = scene.add(
                ConveyorROS(prim_path=self._prim_path, 
                            name=self._name, 
                            position=conveyor_init_position,
                            orientation=conveyor_init_orientation,
                            scale=conveyor_init_scale
        ))
        return