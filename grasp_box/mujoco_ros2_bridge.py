from collections.abc import Callable

import numpy as np
import mujoco
from mujoco import MjModel, MjData

from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, JointState
import cv2

from mujoco_toolbox.mujoco_tools.render_controller import RenderController


class MujocoROS2Bridge(Node):
    """ROS2 bridge for MuJoCo simulation data.
    
    This class is biased towards R1 Pro with grippers.
    """
    
    model: MjModel = property(lambda self: self._model_getter())
    data: MjData = property(lambda self: self._data_getter())

    def __init__(
            self, 
            model_getter: Callable[[], MjModel],
            data_getter: Callable[[], MjData],
            render_controller: RenderController = None,
            camera_names: dict[str, str] = None,
            publish_rate: float = 30.0,  # Hz
            joint_names: dict[str, list[str]] = None  # joint group : joint names
        ):
        super().__init__('mujoco_ros2_bridge')
        
        self._model_getter = model_getter
        self._data_getter = data_getter
        
        self.camera_pubs = {
            'wrist_left': self.create_publisher(
                CompressedImage,
                '/hdas/camera_wrist_left/color/image_raw/compressed',
                10),
            'wrist_right': self.create_publisher(
                CompressedImage,
                '/hdas/camera_wrist_right/color/image_raw/compressed',
                10),
            'head': self.create_publisher(
                CompressedImage,
                '/hdas/camera_head/left_raw/image_raw_color/compressed',
                10),
        }
        
        self.render_controller = render_controller
        self.width = self.render_controller.video_width
        self.height = self.render_controller.video_height
        self.renderer = self.render_controller.renderer
        self.jpeg_quality = 90
        
        # map publisher keys to MuJoCo camera names
        self.camera_names = camera_names or {}
        
        self.arm_left_pub = self.create_publisher(
            JointState, '/hdas/feedback_arm_left', 10)
        self.arm_right_pub = self.create_publisher(
            JointState, '/hdas/feedback_arm_right', 10)
        self.gripper_left_pub = self.create_publisher(
            JointState, '/hdas/feedback_gripper_left', 10)
        self.gripper_right_pub = self.create_publisher(
            JointState, '/hdas/feedback_gripper_right', 10)

        joint_names = joint_names or {}
        self.arm_left_joint_names = joint_names.get('arm_left') or []
        self.arm_right_joint_names = joint_names.get('arm_right') or []
        self.gripper_left_joint_names = joint_names.get('gripper_left') or []
        self.gripper_right_joint_names = joint_names.get('gripper_right') or []
        
        # Log joint configuration
        self.get_logger().info(f'arm_left joints configured: {self.arm_left_joint_names}')
        self.get_logger().info(f'arm_right joints configured: {self.arm_right_joint_names}')
        self.get_logger().info(f'gripper_left joints configured: {self.gripper_left_joint_names}')
        self.get_logger().info(f'gripper_right joints configured: {self.gripper_right_joint_names}')
        
        self.arm_left_joint_ids = self._get_joint_ids(self.arm_left_joint_names)
        self.arm_right_joint_ids = self._get_joint_ids(self.arm_right_joint_names)
        self.gripper_left_joint_ids = self._get_joint_ids(self.gripper_left_joint_names)
        self.gripper_right_joint_ids = self._get_joint_ids(self.gripper_right_joint_names)
        
        # Log resolved joint IDs
        self.get_logger().info(f'arm_left joint IDs resolved: {self.arm_left_joint_ids}')
        self.get_logger().info(f'arm_right joint IDs resolved: {self.arm_right_joint_ids}')
        self.get_logger().info(f'gripper_left joint IDs resolved: {self.gripper_left_joint_ids}')
        self.get_logger().info(f'gripper_right joint IDs resolved: {self.gripper_right_joint_ids}')
        
        # Warn if any joint group has no resolved IDs
        if not self.arm_left_joint_ids:
            self.get_logger().warn('No joint IDs found for arm_left - topic will not publish!')
        if not self.arm_right_joint_ids:
            self.get_logger().warn('No joint IDs found for arm_right - topic will not publish!')
        if not self.gripper_left_joint_ids:
            self.get_logger().warn('No joint IDs found for gripper_left - topic will not publish!')
        if not self.gripper_right_joint_ids:
            self.get_logger().warn('No joint IDs found for gripper_right - topic will not publish!')
        
        self.arm_left_actuator_ids = self._get_actuator_ids(self.arm_left_joint_names)
        self.arm_right_actuator_ids = self._get_actuator_ids(self.arm_right_joint_names)
        
        self.sim_dt = self.model.opt.timestep  # MuJoCo timestep
        self.publish_rate = publish_rate  # Hz
        self.publish_period = 1.0 / self.publish_rate
        self._last_publish_time = 0.0
        
        # NOTE: We don't use a ROS2 timer here because the simulation loop
        # controls timing. Instead, we manually check if it's time to publish.
        # This avoids thread-safety issues with shared MjModel/MjData.
        
        self.get_logger().info('MuJoCo ROS2 Bridge initialized')
    
    def _get_joint_ids(self, joint_names: list = None) -> list:
        """Get MuJoCo joint IDs from joint names."""
        if joint_names is None:
            return []
        joint_ids = []
        for name in joint_names:
            try:
                joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
                if joint_id >= 0:
                    joint_ids.append(joint_id)
                else:
                    self.get_logger().warn(f'Joint not found: {name}')
            except Exception as e:
                self.get_logger().warn(f'Error finding joint {name}: {e}')
        return joint_ids
    
    def _get_actuator_ids(self, joint_names: list = None) -> list:
        """Get MuJoCo actuator IDs for effort/torque readings."""
        if joint_names is None:
            return []
        actuator_ids = []
        for name in joint_names:
            try:
                # Common actuator naming conventions
                for postfix in ['', '_force']:
                    act_name = f'{name}{postfix}'
                    act_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, act_name)
                    if act_id >= 0:
                        actuator_ids.append(act_id)
                        break
                    # For our extraordinary naming of force actuators on grippers
                    act_name = 'rh_A_GJ1_force'
                    act_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, act_name)
                    if act_id >= 0:
                        actuator_ids.append(act_id)
                        break
                    act_name = 'lh_A_GJ1_force'
                    act_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, act_name)
                    if act_id >= 0:
                        actuator_ids.append(act_id)
                        break
            except:
                actuator_ids.append(-1)
        return actuator_ids
    
    def step(self, sim_time: float):
        """Called from the simulation loop to check if publishing is needed.
        
        Args:
            sim_time: Current simulation time in seconds.
        """
        if sim_time - self._last_publish_time >= self.publish_period:
            self.publish_all()
            self._last_publish_time = sim_time
    
    def publish_all(self):
        """Publish all data."""
        stamp = self.get_clock().now().to_msg()
        
        self.publish_cameras(stamp)
        
        self.publish_arm_state(
            self.arm_left_pub,
            stamp,
            self.arm_left_joint_names,
            self.arm_left_joint_ids,
            self.arm_left_actuator_ids,
        )
        self.publish_arm_state(
            self.arm_right_pub,
            stamp,
            self.arm_right_joint_names,
            self.arm_right_joint_ids,
            self.arm_right_actuator_ids,
        )
        self.publish_gripper_state(
            self.gripper_left_pub,
            stamp,
            self.gripper_left_joint_names,
            self.gripper_left_joint_ids,
        )
        self.publish_gripper_state(
            self.gripper_right_pub,
            stamp,
            self.gripper_right_joint_names,
            self.gripper_right_joint_ids,
        )
    
    def publish_cameras(self, stamp):
        """Render and publish all camera images."""
        for key, publisher in self.camera_pubs.items():
            camera_name = self.camera_names.get(key)
            if camera_name:
                try:
                    self.publish_compressed_image(camera_name, publisher, stamp)
                except Exception as e:
                    self.get_logger().warn(f'Camera {camera_name} error: {e}')
    
    def publish_compressed_image(self, camera_name: str, publisher, stamp):
        """Render camera and publish as compressed JPEG."""
        self.renderer.update_scene(self.data, camera=camera_name)
        rgb_array = self.renderer.render()
        
        # Convert RGB to BGR for OpenCV JPEG encoding
        bgr_array = cv2.cvtColor(rgb_array, cv2.COLOR_RGB2BGR)
        
        # Compress to JPEG
        encode_param = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        _, jpeg_data = cv2.imencode('.jpg', bgr_array, encode_param)
        
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = camera_name + '_optical_frame'
        msg.format = 'jpeg'
        msg.data = jpeg_data.tobytes()
        
        publisher.publish(msg)
    
    def publish_arm_state(self, publisher, stamp, joint_names: list = None, joint_ids: list = None, actuator_ids: list = None):
        """Publish arm joint state with position, velocity, and effort."""
        if not joint_ids:
            return
        
        msg = JointState()
        msg.header.stamp = stamp
        msg.header.frame_id = ''
        msg.name = joint_names
        
        positions = []
        velocities = []
        efforts = []
        
        for joint_id in joint_ids:
            if joint_id >= 0:
                qpos_adr = self.model.jnt_qposadr[joint_id]
                qvel_adr = self.model.jnt_dofadr[joint_id]
                
                positions.append(float(self.data.qpos[qpos_adr]))
                velocities.append(float(self.data.qvel[qvel_adr]))
                
                # Get actuator force/torque if available
                # Use qfrc_actuator for the applied actuator forces
                efforts.append(float(self.data.qfrc_actuator[qvel_adr]))
            else:
                positions.append(0.0)
                velocities.append(0.0)
                efforts.append(0.0)
        
        msg.position = positions
        msg.velocity = velocities
        msg.effort = efforts
        
        publisher.publish(msg)
    
    def publish_gripper_state(self, publisher, stamp, joint_names: list = None, joint_ids: list = None):
        """Publish gripper joint state (position only, 0-100mm)."""
        if not joint_ids:
            return
        
        msg = JointState()
        msg.header.stamp = stamp
        msg.header.frame_id = ''
        msg.name = joint_names
        
        positions = []
        for joint_id in joint_ids:
            if joint_id >= 0:
                qpos_adr = self.model.jnt_qposadr[joint_id]
                # Convert to mm since MuJoCo model uses meters
                # Multiply by two since our model has qpos range for each gripper for 5 cm
                pos_meters = float(2 * self.data.qpos[qpos_adr])
                pos_mm = pos_meters * 1000.0  # Convert m to mm
                # Clamp to 0-100mm range
                pos_mm = max(0.0, min(100.0, pos_mm))
                positions.append(pos_mm)
            else:
                positions.append(0.0)
        
        msg.position = positions
        msg.velocity = []  # N/A for gripper
        msg.effort = []    # N/A for gripper
        
        publisher.publish(msg)
