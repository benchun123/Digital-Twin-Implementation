import omni
from omni.isaac.core.controllers import BaseController
from omni.isaac.core.utils.types import ArticulationAction

class SliderController(BaseController):
    def __init__(self):
        super().__init__(name="my_cool_controller")
        return

    # TODO: How to modify controller, it is really fast ...
    def forward(self, command):
        joint_positions = command
        joint_velocities = None
        joint_efforts = None
        return ArticulationAction(joint_positions=joint_positions,joint_efforts=joint_efforts,joint_velocities=joint_velocities)

