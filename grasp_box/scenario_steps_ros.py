"""ROS2-enabled scenario steps for robot control."""

import numpy as np
from typing import Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from grasp_box.mujoco_ros2_bridge import MujocoROS2Bridge


class StepTime:
    """Base class for step timing."""
    pass


class StepTimeNever(StepTime):
    """Step that never ends."""
    pass


class StepTimeRelative(StepTime):
    """Step time relative to another step."""
    def __init__(self, reference_step):
        self.reference_step = reference_step


class ScenarioStep:
    """Base class for scenario steps."""
    def __init__(self):
        self.step_title = ""
        self.start_time = None
        self.end_time = None
    
    def add_to(self, scenario):
        scenario.add_step(self)
        return self


class RobotMoveToPresetPoseStepROS(ScenarioStep):
    """Scenario step that continuously reads target pose from ROS2.
    
    This step reads the target pose from the ROS2 bridge on every simulation step,
    allowing external ROS2 nodes to control the robot in real-time.
    """
    
    def __init__(
            self,
            robot_element_controller,  # RobotController
            ros2_bridge: 'MujocoROS2Bridge',
            arm_name: str,  # "left" or "right"
            start_time: StepTime,
            preset_pose_title: str = None,
    ):
        super().__init__()
        self.robot_element_controller = robot_element_controller
        self.ros2_bridge = ros2_bridge
        self.arm_name = arm_name
        self.start_time = start_time
        self.end_time = StepTimeNever()
        self.step_title = f'{self.robot_element_controller.site_name} controlled by ROS2 "{preset_pose_title}"'
        
        # Optional: fallback pose if no ROS2 command received
        self.fallback_pose: Optional[dict[str, np.ndarray]] = None
        
    def set_fallback_pose(self, fallback_pose: dict[str, np.ndarray]):
        """Set a fallback pose to use when no ROS2 command is available."""
        self.fallback_pose = fallback_pose
        
    def start_action(self, time: float, prev_time: float, dt: float):
        """Called when step starts."""
        # Read current ROS2 command
        preset_pose = self.ros2_bridge.get_arm_control(self.arm_name)
        
        if preset_pose is not None:
            self.robot_element_controller.set_target_pose(preset_pose)
        elif self.fallback_pose is not None:
            self.robot_element_controller.set_target_pose(self.fallback_pose)

    def step_action(self, time: float, prev_time: float, dt: float):
        """Called every simulation step - reads ROS2 command continuously."""
        # Read current ROS2 command (may have changed since last step)
        preset_pose = self.ros2_bridge.get_arm_control(self.arm_name)
        
        if preset_pose is not None:
            self.robot_element_controller.set_target_pose(preset_pose)
        # If no ROS2 command, keep the last target (don't reset to fallback)


class GripperControlStepROS(ScenarioStep):
    """Scenario step that continuously reads gripper commands from ROS2."""
    
    def __init__(
            self,
            gripper_controller,
            ros2_bridge: 'MujocoROS2Bridge',
            gripper_name: str,  # "left" or "right"
            start_time: StepTime,
            title: str = None,
    ):
        super().__init__()
        self.gripper_controller = gripper_controller
        self.ros2_bridge = ros2_bridge
        self.gripper_name = gripper_name
        self.start_time = start_time
        self.end_time = StepTimeNever()
        self.step_title = f'Gripper {gripper_name} controlled by ROS2 "{title}"'
        
        self.fallback_position: Optional[float] = None
        
    def set_fallback_position(self, position_mm: float):
        """Set fallback gripper position (0-100mm)."""
        self.fallback_position = position_mm
        
    def start_action(self, time: float, prev_time: float, dt: float):
        gripper_cmd = self.ros2_bridge.get_gripper_control(self.gripper_name)
        
        if gripper_cmd is not None:
            self.gripper_controller.set_target_pose(gripper_cmd)
        elif self.fallback_position is not None:
            self.gripper_controller.set_position(self.fallback_position)

    def step_action(self, time: float, prev_time: float, dt: float):
        gripper_cmd = self.ros2_bridge.get_gripper_control(self.gripper_name)
        
        if gripper_cmd is not None:
            self.gripper_controller.set_target_pose(gripper_cmd)
