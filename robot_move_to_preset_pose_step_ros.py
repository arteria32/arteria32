import numpy as np
from typing import Optional

# Note: These imports assume these classes exist in your project
# from your_module import ScenarioStep, RobotController, StepTime, StepTimeNever


class RobotMoveToPresetPoseStepROS:
    """A scenario step that moves a robot element to a preset pose."""
    
    def __init__(
            self,
            robot_element_controller,  # RobotController
            start_time,  # StepTime
            preset_pose_title: str = None,
    ):
        super().__init__()
        self.robot_element_controller = robot_element_controller
        self.start_time = start_time
        self.end_time = None  # StepTimeNever()
        self.step_title = f'{self.robot_element_controller.site_name} goes to pose "{preset_pose_title}"'
        self.preset_pose = None  # Initialize preset_pose attribute
        
    def set_preset_pose(self, preset_pose: dict[str, np.ndarray]):
        """Set the preset pose for this step.
        
        Args:
            preset_pose: Dictionary mapping joint names to their target positions.
        """
        self.preset_pose = preset_pose
        
    def start_action(self, time: float, prev_time: float, dt: float):
        if self.preset_pose is not None:
            self.robot_element_controller.set_target_pose(self.preset_pose)
        return

    def step_action(self, time: float, prev_time: float, dt: float):
        # if self.robot_element_controller.is_target_reached():
        #     # rand = 0.05*random.random()
        #     # print(f'random qpos {rand}')
        #     # self.preset_pose =  {
        #     #                         "leap_right/rh_g_j1" : np.array(rand),
        #     #                         "leap_right/rh_g_j2" : np.array(rand),
        #     #                     }
        #     pass
        if self.preset_pose is not None:
            self.robot_element_controller.set_target_pose(self.preset_pose)
        return
