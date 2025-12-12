#!/usr/bin/env python3
"""
Setup OnRobot RG2 Gripper for MuJoCo

This script:
1. Downloads/copies the mesh files to the correct location
2. Creates a working MuJoCo XML file

Usage:
    python3 setup_gripper.py
"""

import os
import shutil
import subprocess
from pathlib import Path


def main():
    script_dir = Path(__file__).parent.absolute()
    
    print("OnRobot RG2 Gripper Setup")
    print("=" * 40)
    print(f"Working directory: {script_dir}")
    
    # Expected mesh directory (relative to XML file)
    mesh_rel_path = "onrobot_ros/onrobot_description/meshes/rg2_v1/visual"
    mesh_dir = script_dir / mesh_rel_path
    
    # Required mesh files
    required_meshes = [
        "body.stl",
        "single_bracket.stl", 
        "moment_arm.stl",
        "truss_arm.stl",
        "finger_tip.stl",
        "flex_finger.stl"
    ]
    
    # Check if meshes exist
    print(f"\nChecking for mesh files in: {mesh_dir}")
    
    missing = []
    for mesh in required_meshes:
        mesh_path = mesh_dir / mesh
        if mesh_path.exists():
            print(f"  ✓ {mesh}")
        else:
            print(f"  ✗ {mesh} - MISSING")
            missing.append(mesh)
    
    if not missing:
        print("\n✓ All mesh files found!")
        print("\nYou can now run:")
        print("  python3 render_gripper.py")
        print("  python3 load_rg2_mujoco.py")
        return
    
    print(f"\n{len(missing)} mesh files missing.")
    
    # Try to download from GitHub
    print("\nAttempting to download meshes from GitHub...")
    
    repo_url = "https://github.com/inria-paris-robotics-lab/onrobot_ros.git"
    clone_dir = script_dir / "onrobot_ros"
    
    if clone_dir.exists():
        print(f"Repository already exists at {clone_dir}")
        # Check if meshes are there
        src_mesh_dir = clone_dir / "onrobot_description/meshes/rg2_v1/visual"
        if src_mesh_dir.exists():
            print("Meshes found in cloned repo!")
        else:
            print("Cloned repo doesn't have meshes. Re-cloning...")
            shutil.rmtree(clone_dir)
            subprocess.run(["git", "clone", "--depth", "1", repo_url, str(clone_dir)], 
                          check=True)
    else:
        print(f"Cloning {repo_url}...")
        try:
            subprocess.run(["git", "clone", "--depth", "1", repo_url, str(clone_dir)], 
                          check=True)
            print("✓ Repository cloned successfully")
        except subprocess.CalledProcessError:
            print("✗ Failed to clone repository")
            print("\nPlease manually download meshes from:")
            print(f"  {repo_url}")
            print(f"\nAnd place them in: {mesh_dir}")
            return
        except FileNotFoundError:
            print("✗ git not found. Please install git or download meshes manually.")
            return
    
    # Verify meshes now exist
    print("\nVerifying mesh files...")
    src_mesh_dir = clone_dir / "onrobot_description/meshes/rg2_v1/visual"
    
    all_found = True
    for mesh in required_meshes:
        if (src_mesh_dir / mesh).exists():
            print(f"  ✓ {mesh}")
        else:
            print(f"  ✗ {mesh} - still missing")
            all_found = False
    
    if all_found:
        print("\n✓ Setup complete!")
        print("\nYou can now run:")
        print("  python3 render_gripper.py")
        print("  python3 load_rg2_mujoco.py --render")
    else:
        print("\n✗ Some meshes are still missing")


def create_standalone_xml():
    """Create XML with meshes in a local 'meshes' folder."""
    
    script_dir = Path(__file__).parent.absolute()
    
    # Create local meshes directory
    local_mesh_dir = script_dir / "meshes" / "rg2"
    local_mesh_dir.mkdir(parents=True, exist_ok=True)
    
    # Source mesh directory
    src_mesh_dir = script_dir / "onrobot_ros/onrobot_description/meshes/rg2_v1/visual"
    
    if not src_mesh_dir.exists():
        print(f"Source meshes not found at: {src_mesh_dir}")
        print("Run setup_gripper.py first to download meshes.")
        return
    
    # Copy meshes
    meshes = ["body.stl", "single_bracket.stl", "moment_arm.stl", 
              "truss_arm.stl", "finger_tip.stl", "flex_finger.stl"]
    
    print(f"Copying meshes to {local_mesh_dir}...")
    for mesh in meshes:
        src = src_mesh_dir / mesh
        dst = local_mesh_dir / mesh
        if src.exists():
            shutil.copy(src, dst)
            print(f"  ✓ {mesh}")
    
    # Read original XML
    xml_path = script_dir / "onrobot_rg2_mujoco.xml"
    with open(xml_path, 'r') as f:
        xml_content = f.read()
    
    # Replace mesh directory
    xml_content = xml_content.replace(
        'meshdir="onrobot_ros/onrobot_description/meshes/rg2_v1/visual"',
        'meshdir="meshes/rg2"'
    )
    
    # Save new XML
    new_xml_path = script_dir / "rg2_gripper.xml"
    with open(new_xml_path, 'w') as f:
        f.write(xml_content)
    
    print(f"\n✓ Created standalone model: {new_xml_path}")
    print(f"  Meshes location: {local_mesh_dir}")


if __name__ == "__main__":
    import sys
    
    if "--standalone" in sys.argv:
        create_standalone_xml()
    else:
        main()
        
        # Also create standalone version
        print("\n" + "=" * 40)
        print("Creating standalone version with local meshes...")
        create_standalone_xml()
