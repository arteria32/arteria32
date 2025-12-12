#!/usr/bin/env python3
"""
Simple RG2 Gripper Renderer - No display required!

Usage:
    python3 render_gripper.py                    # Render images
    python3 render_gripper.py --animate          # Create GIF animation
    python3 render_gripper.py --ctrl 0.5         # Render at specific position
"""

import os
os.environ['MUJOCO_GL'] = 'osmesa'  # Software rendering - always works

import argparse
import numpy as np
from pathlib import Path

import mujoco


def main():
    parser = argparse.ArgumentParser(description='Render RG2 Gripper')
    parser.add_argument('--ctrl', type=float, default=0.65, 
                        help='Gripper control value (0=closed, 1.3=open)')
    parser.add_argument('--animate', action='store_true',
                        help='Create animated GIF')
    parser.add_argument('--output', '-o', type=str, default='gripper_output.png',
                        help='Output filename')
    args = parser.parse_args()
    
    # Find model file - prefer final version
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
        print(f"Error: Model not found!")
        print("Run: python3 setup_gripper.py")
        return
    
    print(f"Loading: {xml_path}")
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    
    # Create renderer
    renderer = mujoco.Renderer(model, height=720, width=960)
    
    # Camera setup
    camera = mujoco.MjvCamera()
    camera.azimuth = 135
    camera.elevation = -25
    camera.distance = 0.5
    camera.lookat[:] = [0, 0, 0.38]
    
    if args.animate:
        # Create animation
        try:
            from PIL import Image
        except ImportError:
            print("Install Pillow for animations: pip install Pillow")
            return
        
        print("Creating animation...")
        images = []
        frames = 60
        
        for i in range(frames):
            t = i / frames * 2 * np.pi
            data.ctrl[0] = 0.65 + 0.65 * np.sin(t)
            
            for _ in range(50):
                mujoco.mj_step(model, data)
            
            renderer.update_scene(data, camera=camera)
            pixels = renderer.render()
            images.append(Image.fromarray(pixels))
            
            if (i + 1) % 20 == 0:
                print(f"  Frame {i+1}/{frames}")
        
        output = args.output if args.output.endswith('.gif') else 'gripper_animation.gif'
        images[0].save(output, save_all=True, append_images=images[1:], 
                       duration=50, loop=0)
        print(f"Saved: {output}")
        
    else:
        # Single image
        try:
            from PIL import Image
        except ImportError:
            print("Install Pillow: pip install Pillow")
            return
        
        # Set gripper position
        data.ctrl[0] = args.ctrl
        for _ in range(500):
            mujoco.mj_step(model, data)
        
        renderer.update_scene(data, camera=camera)
        pixels = renderer.render()
        
        output = args.output if args.output.endswith('.png') else 'gripper_output.png'
        Image.fromarray(pixels).save(output)
        print(f"Saved: {output}")
        print(f"Gripper position: ctrl={args.ctrl:.2f}, joint={data.qpos[0]:.3f} rad")


if __name__ == '__main__':
    main()
