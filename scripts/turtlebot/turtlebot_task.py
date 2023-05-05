from omni.isaac.core.tasks import BaseTask
from omni.isaac.core.scenes.scene import Scene
from omni.isaac.core.utils.stage import add_reference_to_stage, get_stage_units, get_current_stage
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from .turtlebot_ros import TurtlebotROS
import carb
import numpy as np
from typing import Optional


class TurtlebotTask(BaseTask):
    """Task enable turtlebot    
    """
    # def __init__(self, name: str = "turtlebot") -> None:
    def __init__(self, prim_path: str, name: str = "turtlebot", usd_path: Optional[str] = None, position: Optional[np.ndarray] = None, scale: Optional[np.ndarray] = None, orientation: Optional[np.ndarray] = None) -> None:
        BaseTask.__init__(self, name=name, offset=None)
        self._turtlebot_node = None
        self._prim_path = prim_path
        self._name = name
        self._usd_path = usd_path
        self._position = position
        self._orientation = orientation
        self._scale = scale
        carb.log_info("Loaded turtlebot demo")
        print("Loaded turtlebot demo")
    
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

        ## load turtlebot
        # asset_path = base_folder + "src/turtlebot3/urdf_fast/turtlebot3_burger_ros_new.usd"
        # asset_path = base_folder + "src/turtlebot3/urdf_ifl/turtlebot3_burger_ifl_lidar.usd"
        # turtlebot_init_position = np.array([0, 0.2, 0]) / get_stage_units()
        # turtlebot_init_orientation = euler_angles_to_quat(np.array([0, 0, 1.57]))
        # add_reference_to_stage(usd_path=asset_path, prim_path="/World/turtlebot")
        # self._turtlebot_node = self.scene.add(
        #     TurtlebotROS(prim_path="/World/turtlebot", 
        #     name="my_turtlebot", 
        #     position= np.array([0, 0, 0]) ,
        #     wheel_dof_names=["wheel_left_joint", "wheel_right_joint"],
        #     # orientation=turtlebot_init_orientation,
        #     scale=np.array([0.01, 0.01, 0.01])
        # ))

        turtlebot_init_position = self._position / get_stage_units()
        turtlebot_init_orientation = euler_angles_to_quat(self._orientation)
        turtlebot_init_scale = self._scale    

        add_reference_to_stage(usd_path=self._usd_path, prim_path=self._prim_path)
        self._turtlebot_node = scene.add(
                TurtlebotROS(prim_path=self._prim_path, 
                            name=self._name, 
                            position=turtlebot_init_position,
                            orientation=turtlebot_init_orientation,
                            scale=turtlebot_init_scale,
                            wheel_dof_names=["wheel_left_joint", "wheel_right_joint"]
        ))
        
        print("Num of degrees of freedom before first reset: " + str(self._turtlebot_node.num_dof))
        print("World Positions before first reset: " + str(self._turtlebot_node.get_world_pose()))
        print("get_stage_units() " + str(get_stage_units()))

        return


