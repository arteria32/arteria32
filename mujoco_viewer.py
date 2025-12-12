#!/usr/bin/env python3
"""
MuJoCo Interactive Viewer for RG2 Gripper

Usage:
    python3 mujoco_viewer.py
    
If you get OpenGL errors, try:
    1. Make sure you have a display (not SSH without X11)
    2. Install: sudo apt-get install libglfw3 libgl1-mesa-glx
    3. For SSH with X11: ssh -X user@host
"""

import sys
from pathlib import Path

# Don't set MUJOCO_GL for interactive viewer - let it use default (glfw)
import mujoco
import mujoco.viewer
import numpy as np
import time


def main():
    # Find model - prefer final version
    script_dir = Path(__file__).parent
    
    # Try final version first (correct positions)
    xml_path = script_dir / "rg2_gripper_final.xml"
    if not xml_path.exists():
        xml_path = script_dir / "rg2_gripper_v5.xml"
    if not xml_path.exists():
        xml_path = script_dir / "rg2_gripper.xml"
    if not xml_path.exists():
        xml_path = script_dir / "onrobot_rg2_mujoco.xml"
    
    if not xml_path.exists():
        print("Error: Model file not found!")
        print("Run: python3 setup_gripper.py")
        return
    
    print(f"Loading: {xml_path}")
    
    try:
        model = mujoco.MjModel.from_xml_path(str(xml_path))
        data = mujoco.MjData(model)
    except Exception as e:
        print(f"Error loading model: {e}")
        return
    
    print(f"Model loaded: {model.nbody} bodies, {model.njnt} joints")
    print()
    print("=" * 50)
    print("VIEWER CONTROLS:")
    print("=" * 50)
    print("  Mouse Left:    Rotate camera")
    print("  Mouse Right:   Pan camera")  
    print("  Scroll:        Zoom in/out")
    print("  Double-click:  Track/select body")
    print("  Space:         Pause/Resume simulation")
    print("  Backspace:     Reset simulation")
    print("  ESC:           Exit")
    print("=" * 50)
    print()
    print("Launching viewer...")
    print("(Gripper will animate automatically)")
    print()
    
    # Set initial gripper position
    data.ctrl[0] = 0.5
    for _ in range(100):
        mujoco.mj_step(model, data)
    
    # Launch viewer
    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            # Set nice camera angle
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -25
            viewer.cam.distance = 0.5
            viewer.cam.lookat[:] = [0, 0, 0.38]
            
            start_time = time.time()
            
            while viewer.is_running():
                # Animate gripper open/close
                t = time.time() - start_time
                data.ctrl[0] = 0.65 + 0.65 * np.sin(t * 1.5)
                
                # Step simulation
                mujoco.mj_step(model, data)
                
                # Sync viewer
                viewer.sync()
                
                # Small delay to not overwhelm CPU
                time.sleep(0.01)
                
    except Exception as e:
        print(f"\nViewer error: {e}")
        print()
        print("TROUBLESHOOTING:")
        print("-" * 40)
        print("1. Make sure you're running on a machine with a display")
        print("   (not a headless server via SSH)")
        print()
        print("2. If using SSH, enable X11 forwarding:")
        print("   ssh -X user@hostname")
        print()
        print("3. Install OpenGL libraries:")
        print("   sudo apt-get install libglfw3 libgl1-mesa-glx")
        print()
        print("4. If on a server, use render_gripper.py instead:")
        print("   python3 render_gripper.py")


if __name__ == "__main__":
    main()
