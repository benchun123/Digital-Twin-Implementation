from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.controllers import BaseController

import numpy as np


class SliderController(BaseController):
    def __init__(self):
        super().__init__(name="my_cool_controller")
        self.name = "test"
        return

    def forward(self,
        start_position: np.ndarray,
        goal_position: np.ndarray,
        joint_velocity: float = 1.0,
        position_tol: float = 0.005,
    ):
        """[summary]
        Args:
            start_position (np.ndarray): [description]
            goal_position (np.ndarray): [description]
            joint_velocity (float, optional): [description]. Defaults to 0.5.
            position_tol (float, optional): [description]. Defaults to 4.0.

        Returns:
            ArticulationAction: [description]
        """
        if np.abs(goal_position[0] - start_position[0]) < position_tol:
            command = [0.0]
        elif goal_position[0] > start_position[0]:
            command = [joint_velocity]
        else:
            command = [-joint_velocity]

        return ArticulationAction(joint_velocities=command)

