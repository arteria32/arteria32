#!/usr/bin/env python3
"""
OnRobot RG2 Gripper MuJoCo Loader

This script demonstrates how to load and control the RG2 gripper in MuJoCo.
It can also convert the URDF xacro to a standard URDF if needed.

Usage:
    python load_rg2_mujoco.py              # Load and test the gripper
    python load_rg2_mujoco.py --render     # Load with visualization (requires display)
    python load_rg2_mujoco.py --convert    # Convert xacro to URDF
"""

import os
import sys

# Set MuJoCo GL backend BEFORE importing mujoco
# Options: 'egl' (GPU), 'osmesa' (software), 'glfw' (display)
if 'MUJOCO_GL' not in os.environ:
    # Try to auto-detect the best backend
    if os.environ.get('DISPLAY'):
        os.environ['MUJOCO_GL'] = 'glfw'  # Has display
    else:
        os.environ['MUJOCO_GL'] = 'osmesa'  # Headless

import argparse
import numpy as np
from pathlib import Path
import subprocess


def install_mujoco():
    """Install MuJoCo if not available."""
    try:
        import mujoco
        return True
    except ImportError:
        print("Installing mujoco...")
        subprocess.check_call([sys.executable, '-m', 'pip', 'install', 'mujoco', '-q'])
        return True


def load_gripper_model(xml_path: str = None):
    """
    Load the RG2 gripper model in MuJoCo.
    
    Args:
        xml_path: Path to the MuJoCo XML file. If None, uses default.
        
    Returns:
        model: MuJoCo model
        data: MuJoCo data
    """
    import mujoco
    
    if xml_path is None:
        xml_path = Path(__file__).parent / "onrobot_rg2_mujoco.xml"
    
    print(f"Loading model from: {xml_path}")
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    
    print(f"Model loaded successfully!")
    print(f"  - Bodies: {model.nbody}")
    print(f"  - Joints: {model.njnt}")
    print(f"  - Actuators: {model.nu}")
    print(f"  - Timestep: {model.opt.timestep}s")
    
    return model, data


def print_model_info(model):
    """Print detailed information about the model."""
    import mujoco
    
    print("\n=== Model Information ===")
    
    print("\nBodies:")
    for i in range(model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
        print(f"  {i}: {name}")
    
    print("\nJoints:")
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        jnt_type = ['free', 'ball', 'slide', 'hinge'][model.jnt_type[i]]
        range_low = model.jnt_range[i, 0]
        range_high = model.jnt_range[i, 1]
        print(f"  {i}: {name} (type={jnt_type}, range=[{range_low:.3f}, {range_high:.3f}])")
    
    print("\nActuators:")
    for i in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        ctrl_range = model.actuator_ctrlrange[i]
        print(f"  {i}: {name} (ctrlrange=[{ctrl_range[0]:.3f}, {ctrl_range[1]:.3f}])")
    
    print("\nGeoms:")
    for i in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
        geom_type = ['plane', 'hfield', 'sphere', 'capsule', 'ellipsoid', 
                     'cylinder', 'box', 'mesh'][model.geom_type[i]]
        print(f"  {i}: {name} (type={geom_type})")


def control_gripper(model, data, position: float, steps: int = 100):
    """
    Control the gripper to a target position.
    
    Args:
        model: MuJoCo model
        data: MuJoCo data
        position: Target gripper position (0=closed, 1.3=fully open)
        steps: Number of simulation steps
    """
    import mujoco
    
    # Set control input
    data.ctrl[0] = position
    
    # Simulate
    for _ in range(steps):
        mujoco.mj_step(model, data)
    
    # Get current gripper position
    gripper_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "gripper_joint")
    current_pos = data.qpos[model.jnt_qposadr[gripper_joint_id]]
    
    return current_pos


def test_gripper_motion(model, data):
    """Test opening and closing the gripper."""
    import mujoco
    
    print("\n=== Testing Gripper Motion ===")
    
    # Reset to initial state
    mujoco.mj_resetData(model, data)
    
    # Open gripper
    print("Opening gripper...")
    final_pos = control_gripper(model, data, 1.0, steps=500)
    print(f"  Position after opening: {final_pos:.3f} rad")
    
    # Close gripper
    print("Closing gripper...")
    final_pos = control_gripper(model, data, 0.0, steps=500)
    print(f"  Position after closing: {final_pos:.3f} rad")
    
    # Partial open
    print("Moving to 50%...")
    final_pos = control_gripper(model, data, 0.65, steps=500)
    print(f"  Position at 50%: {final_pos:.3f} rad")
    
    print("\n✓ Gripper motion test passed!")


def render_gripper(model, data, duration: float = 5.0):
    """
    Render the gripper with interactive visualization.
    
    Requires a display. Will animate the gripper opening/closing.
    """
    import mujoco
    import mujoco.viewer
    
    print("\n=== Rendering Gripper ===")
    print("Opening viewer... (close window to exit)")
    print("The gripper will animate open/close automatically.")
    
    # Launch viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Animate for specified duration
        start_time = data.time
        
        while viewer.is_running() and (data.time - start_time) < duration:
            # Oscillate gripper position
            t = data.time - start_time
            position = 0.65 + 0.65 * np.sin(t * 2)  # Oscillate between 0 and 1.3
            data.ctrl[0] = position
            
            # Step simulation
            mujoco.mj_step(model, data)
            
            # Sync viewer
            viewer.sync()
        
        print("Viewer closed.")


def convert_xacro_to_urdf(output_path: str = None):
    """
    Convert the ROS xacro files to a standard URDF.
    
    Requires: ros2, xacro package
    
    Note: MuJoCo can load URDF directly, but xacro needs preprocessing.
    """
    workspace = Path(__file__).parent
    xacro_file = workspace / "onrobot_ros/onrobot_description/urdf/test.urdf.xacro"
    
    if not xacro_file.exists():
        print(f"Error: Xacro file not found: {xacro_file}")
        return None
    
    if output_path is None:
        output_path = workspace / "onrobot_rg2.urdf"
    
    print(f"Converting xacro to URDF...")
    print(f"  Input: {xacro_file}")
    print(f"  Output: {output_path}")
    
    try:
        # Try using xacro command
        result = subprocess.run(
            ['xacro', str(xacro_file), 'model:=rg2_v1'],
            capture_output=True,
            text=True
        )
        
        if result.returncode == 0:
            with open(output_path, 'w') as f:
                f.write(result.stdout)
            print(f"✓ URDF saved to: {output_path}")
            return output_path
        else:
            print(f"xacro failed: {result.stderr}")
            return None
            
    except FileNotFoundError:
        print("Error: 'xacro' command not found.")
        print("Install with: pip install xacro")
        print("Or use ROS2: sudo apt install ros-<distro>-xacro")
        return None


def load_urdf_in_mujoco(urdf_path: str):
    """
    Load a URDF file directly in MuJoCo.
    
    MuJoCo can load URDF files, but with some limitations:
    - No support for mimic joints (need to use equality constraints)
    - May need mesh path adjustments
    """
    import mujoco
    
    print(f"Loading URDF: {urdf_path}")
    
    try:
        model = mujoco.MjModel.from_xml_path(urdf_path)
        data = mujoco.MjData(model)
        print("✓ URDF loaded successfully!")
        return model, data
    except Exception as e:
        print(f"Failed to load URDF: {e}")
        print("\nNote: MuJoCo URDF support has limitations.")
        print("Use the native MuJoCo XML (onrobot_rg2_mujoco.xml) instead.")
        return None, None


def create_gripper_controller():
    """
    Example: Create a simple gripper controller class.
    """
    import mujoco
    
    class RG2GripperController:
        """Controller for the OnRobot RG2 gripper in MuJoCo."""
        
        def __init__(self, model, data):
            self.model = model
            self.data = data
            self.actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "gripper_ctrl")
            self.joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "gripper_joint")
            
            # Gripper limits
            self.min_pos = 0.0      # Closed
            self.max_pos = 1.3     # Fully open
            
        @property
        def position(self):
            """Current gripper joint position in radians."""
            return self.data.qpos[self.model.jnt_qposadr[self.joint_id]]
        
        @property
        def width(self):
            """Approximate gripper opening width in meters."""
            # Approximate conversion from joint angle to width
            # RG2 has ~110mm stroke
            return (self.position / self.max_pos) * 0.110
        
        def open(self, amount: float = 1.0):
            """Open the gripper (0.0 to 1.0)."""
            self.data.ctrl[self.actuator_id] = amount * self.max_pos
            
        def close(self):
            """Close the gripper."""
            self.data.ctrl[self.actuator_id] = self.min_pos
            
        def set_position(self, position: float):
            """Set gripper position directly (0.0 to 1.3 rad)."""
            self.data.ctrl[self.actuator_id] = np.clip(position, self.min_pos, self.max_pos)
            
        def set_width(self, width: float):
            """Set gripper opening width in meters (0.0 to 0.11m)."""
            position = (width / 0.110) * self.max_pos
            self.set_position(position)
            
        def step(self, n_steps: int = 1):
            """Step the simulation."""
            for _ in range(n_steps):
                mujoco.mj_step(self.model, self.data)
    
    return RG2GripperController


def main():
    parser = argparse.ArgumentParser(description='OnRobot RG2 MuJoCo Loader')
    parser.add_argument('--render', action='store_true', 
                        help='Render the gripper with visualization')
    parser.add_argument('--convert', action='store_true',
                        help='Convert xacro to URDF')
    parser.add_argument('--info', action='store_true',
                        help='Print detailed model information')
    parser.add_argument('--xml', type=str, default=None,
                        help='Path to MuJoCo XML file')
    
    args = parser.parse_args()
    
    # Install MuJoCo if needed
    install_mujoco()
    
    if args.convert:
        convert_xacro_to_urdf()
        return
    
    # Load the model
    try:
        model, data = load_gripper_model(args.xml)
    except Exception as e:
        print(f"Error loading model: {e}")
        print("\nMake sure the mesh files are in the correct location.")
        print("Expected path: onrobot_ros/onrobot_description/meshes/rg2_v1/visual/")
        return
    
    if args.info:
        print_model_info(model)
    
    # Test gripper motion
    test_gripper_motion(model, data)
    
    # Render if requested
    if args.render:
        try:
            render_gripper(model, data, duration=10.0)
        except Exception as e:
            print(f"Rendering failed: {e}")
            print("Note: Rendering requires a display and mujoco viewer.")
    
    # Demo the controller class
    print("\n=== Controller Demo ===")
    RG2Controller = create_gripper_controller()
    controller = RG2Controller(model, data)
    
    print(f"Initial position: {controller.position:.3f} rad")
    print(f"Initial width: {controller.width*1000:.1f} mm")
    
    controller.open(0.5)
    controller.step(200)
    print(f"After 50% open: {controller.width*1000:.1f} mm")
    
    controller.set_width(0.05)  # 50mm opening
    controller.step(200)
    print(f"After set_width(0.05): {controller.width*1000:.1f} mm")


if __name__ == '__main__':
    main()
