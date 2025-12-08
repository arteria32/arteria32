"""Dependencies module - creates and manages all simulation components."""

from collections.abc import Callable
from typing import TYPE_CHECKING

from grasp_box.launch_options import LaunchOptions
from grasp_box.mujoco_ros2_bridge import MujocoROS2Bridge

if TYPE_CHECKING:
    from mujoco import MjModel, MjData


class Dependencies:
    """Container for all simulation dependencies.
    
    This class creates and holds references to all controllers, the launcher,
    and the ROS2 bridge. The MuJoCo model and data are shared between the
    launcher and the ROS2 bridge.
    """
    
    def __init__(
        self,
        step: Callable[[float, float], None],
        launch_options: LaunchOptions
    ):
        self.step = step
        self.launch_options = launch_options
        
        # Initialize the MuJoCo launcher first (it creates model and data)
        self.launcher = self._create_launcher()
        
        # Initialize all controllers...
        self._init_controllers()
        
        # Initialize ROS2 bridge AFTER rclpy.init() has been called in main.py
        # The bridge shares the same model/data as the launcher
        self.mujoco_ROS2_bridge = self._create_ros2_bridge()
    
    def _create_launcher(self):
        """Create the MuJoCo launcher."""
        # Your existing launcher creation code
        # This is a placeholder - replace with your actual implementation
        from mujoco_toolbox.launcher import Launcher  # Example import
        return Launcher(
            step_callback=self.step,
            # ... other parameters
        )
    
    def _init_controllers(self):
        """Initialize all controllers."""
        # Your existing controller initialization code
        # This is a placeholder - replace with your actual implementations
        
        # Example placeholders for all your controllers:
        self.scenario = None
        self.target_left_mocap_controller = _DummyController()
        self.target_right_mocap_controller = _DummyController()
        self.cart_pos_ctrl = _DummyController()
        self.robot_body_inverse_dynamics_ctrl = _DummyController()
        self.body_pos_ctrl = _DummyController()
        self.body_up_pos_ctrl = _DummyController()
        self.torso_pos_ctrl = _DummyController()
        self.torso_up_pos_ctrl = _DummyController()
        self.torso_down_pos_ctrl = _DummyController()
        self.arm_left_tree_pos_ctrl = _DummyController()
        self.arm_left_pos_ctrl = _DummyController()
        self.arm_right_tree_pos_ctrl = _DummyController()
        self.arm_right_pos_ctrl = _DummyController()
        self.hand_left_pos_ctrl = _DummyController()
        self.lh_tf_pos_ctrl = _DummyController()
        self.lh_pf_pos_ctrl = _DummyController()
        self.lh_mf_pos_ctrl = _DummyController()
        self.lh_rf_pos_ctrl = _DummyController()
        self.lh_lf_pos_ctrl = _DummyController()
        self.hand_right_pos_ctrl = _DummyController()
        self.rh_tf_pos_ctrl = _DummyController()
        self.rh_pf_pos_ctrl = _DummyController()
        self.rh_mf_pos_ctrl = _DummyController()
        self.rh_rf_pos_ctrl = _DummyController()
        self.rh_lf_pos_ctrl = _DummyController()
        
        self.body_impedance_ctrl = _DummyController()
        self.body_up_impedance_ctrl = _DummyController()
        self.torso_impedance_ctrl = _DummyController()
        self.torso_up_impedance_ctrl = _DummyController()
        self.torso_down_impedance_ctrl = _DummyController()
        self.arm_left_tree_impedance_ctrl = _DummyController()
        self.arm_left_impedance_ctrl = _DummyController()
        self.arm_right_tree_impedance_ctrl = _DummyController()
        self.arm_right_impedance_ctrl = _DummyController()
        self.hand_left_impedance_ctrl = _DummyController()
        self.lh_tf_impedance_ctrl = _DummyController()
        self.lh_pf_impedance_ctrl = _DummyController()
        self.lh_mf_impedance_ctrl = _DummyController()
        self.lh_rf_impedance_ctrl = _DummyController()
        self.lh_lf_impedance_ctrl = _DummyController()
        self.hand_right_impedance_ctrl = _DummyController()
        self.rh_tf_impedance_ctrl = _DummyController()
        self.rh_pf_impedance_ctrl = _DummyController()
        self.rh_mf_impedance_ctrl = _DummyController()
        self.rh_rf_impedance_ctrl = _DummyController()
        self.rh_lf_impedance_ctrl = _DummyController()
        
        self.camera_controller = _DummyController()
        self.joints_limits_controller = _DummyController()
        self.fire_equality_controller = _DummyController()
    
    def _create_ros2_bridge(self) -> MujocoROS2Bridge:
        """Create the ROS2 bridge for publishing MuJoCo data.
        
        IMPORTANT: This must be called AFTER rclpy.init() in main.py
        """
        # Define camera name mappings (publisher key -> MuJoCo camera name)
        camera_names = {
            'wrist_left': 'camera_wrist_left',    # Replace with your actual camera names
            'wrist_right': 'camera_wrist_right',
            'head': 'camera_head',
        }
        
        # Define joint name mappings (group -> list of joint names)
        joint_names = {
            'arm_left': [
                'arm_left_joint_1',
                'arm_left_joint_2',
                'arm_left_joint_3',
                'arm_left_joint_4',
                'arm_left_joint_5',
                'arm_left_joint_6',
                'arm_left_joint_7',
            ],
            'arm_right': [
                'arm_right_joint_1',
                'arm_right_joint_2',
                'arm_right_joint_3',
                'arm_right_joint_4',
                'arm_right_joint_5',
                'arm_right_joint_6',
                'arm_right_joint_7',
            ],
            'gripper_left': ['gripper_left_joint'],
            'gripper_right': ['gripper_right_joint'],
        }
        
        return MujocoROS2Bridge(
            model_getter=lambda: self.launcher.model,  # Shared model
            data_getter=lambda: self.launcher.data,    # Shared data
            render_controller=self.launcher.render_controller,
            camera_names=camera_names,
            publish_rate=30.0,  # Hz - adjust as needed
            joint_names=joint_names,
        )


class _DummyController:
    """Placeholder controller for demonstration."""
    def step(self, dt: float):
        pass
