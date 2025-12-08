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

## ROS2 Topics Published

| Topic | Type | Description |
|-------|------|-------------|
| `/hdas/camera_wrist_left/color/image_raw/compressed` | `CompressedImage` | Left wrist camera |
| `/hdas/camera_wrist_right/color/image_raw/compressed` | `CompressedImage` | Right wrist camera |
| `/hdas/camera_head/left_raw/image_raw_color/compressed` | `CompressedImage` | Head camera |
| `/hdas/feedback_arm_left` | `JointState` | Left arm joint state |
| `/hdas/feedback_arm_right` | `JointState` | Right arm joint state |
| `/hdas/feedback_gripper_left` | `JointState` | Left gripper state |
| `/hdas/feedback_gripper_right` | `JointState` | Right gripper state |

## Usage

```bash
# Basic run
python main.py

# With arguments
python main.py --headless_mode --collect_data

# Check ROS2 topics
ros2 topic list
ros2 topic echo /hdas/feedback_arm_left
```

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
