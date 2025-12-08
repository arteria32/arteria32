# MuJoCo ROS2 Bridge Integration

This project integrates a MuJoCo simulation with ROS2 for publishing sensor data.

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                         main.py                                  │
│  ┌─────────────────────────────────────────────────────────────┐│
│  │  1. rclpy.init()  ← Initialize ROS2 first                   ││
│  │  2. Dependencies()  ← Creates launcher & ROS2 bridge        ││
│  │  3. launcher.start()  ← Runs simulation loop                ││
│  │  4. rclpy.shutdown()  ← Cleanup on exit                     ││
│  └─────────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                    Simulation Loop (launcher)                    │
│  ┌─────────────────────────────────────────────────────────────┐│
│  │  for each timestep:                                         ││
│  │    mujoco.mj_step(model, data)                              ││
│  │    step(time, dt)  ← Your step function                     ││
│  └─────────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                       step() function                            │
│  ┌─────────────────────────────────────────────────────────────┐│
│  │  1. Update all controllers                                  ││
│  │  2. mujoco_ROS2_bridge.step(time)  ← Publish if due         ││
│  │  3. rclpy.spin_once(node, timeout_sec=0)  ← Process ROS2    ││
│  └─────────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────────┘
```

## Key Design Decisions

### Why `spin_once()` instead of `spin()`?

| Aspect | `spin_once()` | `spin()` in thread |
|--------|---------------|-------------------|
| Thread safety | ✅ Single-threaded, safe | ⚠️ Requires locks for MjModel/MjData |
| Complexity | Simple | More complex |
| Timing control | Simulation controls timing | ROS2 controls timing |
| Debugging | Easy | Harder (multi-threaded) |

Since the MuJoCo `launcher` and `mujoco_ROS2_bridge` share the same `MjModel` and `MjData`, using `spin_once()` in the simulation loop is **safer and simpler**.

### Why not use ROS2 timers?

The `MujocoROS2Bridge` does NOT use `self.create_timer()` because:
1. The simulation loop already controls timing
2. Timer callbacks would only fire during `spin_once()` anyway
3. Manual rate control in `bridge.step(sim_time)` is more predictable

## File Structure

```
workspace/
├── main.py                          # Entry point with ROS2 init/shutdown
└── grasp_box/
    ├── __init__.py
    ├── launch_options.py            # Configuration constants
    ├── dependencies.py              # Creates all components
    ├── mujoco_ros2_bridge.py        # ROS2 node for publishing data
    └── scenario.py                  # Scenario setup
```

## ROS2 Topics

### Published (Feedback from Simulation)

| Topic | Type | Description |
|-------|------|-------------|
| `/hdas/camera_wrist_left/color/image_raw/compressed` | `CompressedImage` | Left wrist camera |
| `/hdas/camera_wrist_right/color/image_raw/compressed` | `CompressedImage` | Right wrist camera |
| `/hdas/camera_head/left_raw/image_raw_color/compressed` | `CompressedImage` | Head camera |
| `/hdas/feedback_arm_left` | `JointState` | Left arm joint state |
| `/hdas/feedback_arm_right` | `JointState` | Right arm joint state |
| `/hdas/feedback_gripper_left` | `JointState` | Left gripper state |
| `/hdas/feedback_gripper_right` | `JointState` | Right gripper state |

### Subscribed (Control Inputs to Simulation)

| Topic | Type | Description |
|-------|------|-------------|
| `/motion_control/control_arm_left` | `JointState` | Left arm position commands |
| `/motion_control/control_arm_right` | `JointState` | Right arm position commands |
| `/motion_control/control_gripper_left` | `JointState` | Left gripper position (0-100mm) |
| `/motion_control/control_gripper_right` | `JointState` | Right gripper position (0-100mm) |

### Control Message Format (JointState)

Using standard `sensor_msgs/JointState` instead of custom `hdas_msg::msg::MotorControl`:

**Arm Control:**
```python
# JointState message for arm control
msg.position = [j1, j2, j3, j4, j5, j6, j7]  # p_des - 7 joint positions in radians
msg.velocity = []  # v_des - not used
msg.effort = []    # t_ff - not used
```

**Gripper Control:**
```python
# JointState message for gripper control
msg.position = [gripper_pos]  # p_des - gripper position 0-100mm
msg.velocity = []  # v_des - not used
msg.effort = []    # t_ff - not used
```

### Example: Sending Control Commands

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        self.arm_left_pub = self.create_publisher(
            JointState, '/motion_control/control_arm_left', 10)
        self.gripper_left_pub = self.create_publisher(
            JointState, '/motion_control/control_gripper_left', 10)
    
    def send_arm_command(self, positions: list):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = positions  # [j1, j2, j3, j4, j5, j6, j7]
        self.arm_left_pub.publish(msg)
    
    def send_gripper_command(self, position_mm: float):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = [position_mm]  # 0-100mm
        self.gripper_left_pub.publish(msg)
```

## Usage

```bash
# Basic run (simulation only, no ROS2, with GUI)
python main.py

# Enable ROS2 publishing (automatically enables headless_mode, render_video, finish_scenario)
python main.py --publish_ros2

# ROS2 mode with data collection
python main.py --publish_ros2 --collect_data

# Explicitly disable ROS2 (default)
python main.py --no_publish_ros2

# Check ROS2 topics (when publish_ros2 is enabled)
ros2 topic list
ros2 topic echo /hdas/feedback_arm_left
```

### ROS2 Mode Auto-Enabled Flags

When `--publish_ros2` is set, the following flags are **automatically enabled**:

| Flag | Auto-set to | Reason |
|------|-------------|--------|
| `--headless_mode` | `True` | Run without GUI (for servers/containers) |
| `--render_video` | `True` | Required for camera image publishing |
| `--finish_scenario` | `True` | Auto-finish scenario when complete |

### Additional Flags

| Flag | Default | Description |
|------|---------|-------------|
| `--collect_data` | `False` | Enable data collection (not auto-enabled) |
| `--scenario_name` | `kitchen_oven` | Scenario to run |
| `--box_name` | `r307` | Box to use |
| `--camera_scenario` | `third_person` | Camera POV |

## Configuration

Adjust publishing rate in `dependencies.py`:

```python
return MujocoROS2Bridge(
    ...
    publish_rate=30.0,  # Hz - adjust as needed
    ...
)
```

## Thread Safety Notes

- **Safe**: Reading `model` and `data` during `step()` (same thread as `mj_step`)
- **Safe**: Publishing ROS2 messages from `step()` 
- **Safe**: `spin_once()` with `timeout_sec=0` (non-blocking)
- **NOT safe**: Accessing `model`/`data` from a separate ROS2 thread (don't do this)
