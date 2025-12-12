#!/usr/bin/env python3
"""
OnRobot RG2 Gripper Viewer for MuJoCo

Usage:
    # Render images (works without display)
    MUJOCO_GL=osmesa python view_gripper.py
    
    # Interactive viewer (requires display)
    python view_gripper.py --interactive
    
    # Create animation GIF
    MUJOCO_GL=osmesa python view_gripper.py --animate
"""

import os
import sys
import time
import argparse
import numpy as np
from pathlib import Path

# Set default GL backend for headless rendering
if 'MUJOCO_GL' not in os.environ:
    os.environ['MUJOCO_GL'] = 'osmesa'

import mujoco


def load_model():
    """Load the RG2 gripper model."""
    xml_path = Path(__file__).parent / "onrobot_rg2_mujoco.xml"
    
    if not xml_path.exists():
        print(f"Error: Model file not found: {xml_path}")
        sys.exit(1)
    
    print(f"Loading model: {xml_path}")
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    
    print(f"Model loaded: {model.nbody} bodies, {model.njnt} joints, {model.nmesh} meshes")
    return model, data


def render_images(model, data):
    """Render the gripper in different states."""
    from PIL import Image
    
    renderer = mujoco.Renderer(model, height=720, width=960)
    
    camera = mujoco.MjvCamera()
    camera.azimuth = 135
    camera.elevation = -25
    camera.distance = 0.6
    camera.lookat[:] = [0, 0, 0.35]
    
    def render_state(ctrl_val, filename):
        data.ctrl[0] = ctrl_val
        for _ in range(500):
            mujoco.mj_step(model, data)
        renderer.update_scene(data, camera=camera)
        pixels = renderer.render()
        Image.fromarray(pixels).save(filename)
        print(f"Saved: {filename}")
    
    render_state(0.0, 'gripper_closed.png')
    render_state(0.65, 'gripper_half.png')
    render_state(1.3, 'gripper_open.png')
    
    print("\nRendered images saved!")


def create_animation(model, data, output='gripper_animation.gif', frames=60):
    """Create an animated GIF of the gripper opening/closing."""
    from PIL import Image
    
    renderer = mujoco.Renderer(model, height=480, width=640)
    
    camera = mujoco.MjvCamera()
    camera.azimuth = 135
    camera.elevation = -25
    camera.distance = 0.5
    camera.lookat[:] = [0, 0, 0.38]
    
    images = []
    
    print(f"Creating animation with {frames} frames...")
    
    # Reset
    mujoco.mj_resetData(model, data)
    
    for i in range(frames):
        # Oscillate gripper
        t = i / frames * 2 * np.pi
        ctrl = 0.65 + 0.65 * np.sin(t)
        data.ctrl[0] = ctrl
        
        # Simulate
        for _ in range(50):
            mujoco.mj_step(model, data)
        
        # Render
        renderer.update_scene(data, camera=camera)
        pixels = renderer.render()
        images.append(Image.fromarray(pixels))
        
        if (i + 1) % 10 == 0:
            print(f"  Frame {i + 1}/{frames}")
    
    # Save GIF
    images[0].save(
        output,
        save_all=True,
        append_images=images[1:],
        duration=50,  # ms per frame
        loop=0
    )
    print(f"Saved animation: {output}")


def interactive_viewer(model, data):
    """Launch interactive viewer (requires display)."""
    import mujoco.viewer
    
    print("\n=== Interactive Viewer ===")
    print("Controls:")
    print("  Mouse drag: Rotate camera")
    print("  Scroll: Zoom")
    print("  Double-click: Track body")
    print("  Space: Pause/Resume")
    print("  ESC: Exit")
    print("\nLaunching viewer...")
    
    # Set initial position
    data.ctrl[0] = 0.5
    for _ in range(100):
        mujoco.mj_step(model, data)
    
    target = 0.5
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -25
        viewer.cam.distance = 0.5
        viewer.cam.lookat[:] = [0, 0, 0.35]
        
        start = time.time()
        
        while viewer.is_running():
            # Animate gripper
            t = time.time() - start
            data.ctrl[0] = 0.65 + 0.65 * np.sin(t * 2)
            
            mujoco.mj_step(model, data)
            viewer.sync()
            
            time.sleep(model.opt.timestep)
    
    print("Viewer closed.")


def main():
    parser = argparse.ArgumentParser(description='RG2 Gripper Viewer')
    parser.add_argument('--interactive', '-i', action='store_true',
                        help='Launch interactive viewer (requires display)')
    parser.add_argument('--animate', '-a', action='store_true',
                        help='Create animated GIF')
    parser.add_argument('--frames', type=int, default=60,
                        help='Number of frames for animation (default: 60)')
    
    args = parser.parse_args()
    
    model, data = load_model()
    
    if args.interactive:
        try:
            interactive_viewer(model, data)
        except Exception as e:
            print(f"Interactive viewer failed: {e}")
            print("Try running without --interactive to render images instead.")
    elif args.animate:
        try:
            from PIL import Image
            create_animation(model, data, frames=args.frames)
        except ImportError:
            print("PIL required for animation. Install with: pip install Pillow")
    else:
        try:
            from PIL import Image
            render_images(model, data)
        except ImportError:
            print("PIL required for rendering. Install with: pip install Pillow")


if __name__ == '__main__':
    main()
