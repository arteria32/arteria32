#!/usr/bin/env python3
"""
STL Splitter for MuJoCo Gripper Simulation

This script splits a single STL file containing gripper parts into separate STL files
that can be used as individual joints in MuJoCo simulation.

Usage:
    python split_stl.py <input.stl> [options]

Methods available:
    1. connected_components: Split by disconnected mesh parts (default)
    2. plane: Split by a cutting plane (e.g., x=0 to split left/right)
    3. interactive: Visualize and manually select split parameters
"""

import argparse
import numpy as np
from pathlib import Path


def install_dependencies():
    """Install required packages if not available."""
    import subprocess
    import sys
    
    packages = ['trimesh', 'numpy']
    for package in packages:
        try:
            __import__(package)
        except ImportError:
            print(f"Installing {package}...")
            subprocess.check_call([sys.executable, '-m', 'pip', 'install', package, '-q'])


def load_mesh(filepath):
    """Load an STL file using trimesh."""
    import trimesh
    mesh = trimesh.load(filepath)
    print(f"Loaded mesh: {filepath}")
    print(f"  - Vertices: {len(mesh.vertices)}")
    print(f"  - Faces: {len(mesh.faces)}")
    print(f"  - Bounds: {mesh.bounds}")
    print(f"  - Center: {mesh.centroid}")
    return mesh


def split_by_connected_components(mesh):
    """
    Split mesh into connected components.
    This works when the gripper parts are physically separate in the mesh.
    """
    import trimesh
    
    # Split into connected components
    components = mesh.split(only_watertight=False)
    
    if len(components) == 1:
        print("Warning: Mesh has only one connected component.")
        print("Try using --method plane to split by a cutting plane instead.")
        return components
    
    print(f"Found {len(components)} connected components:")
    for i, comp in enumerate(components):
        print(f"  Component {i}: {len(comp.vertices)} vertices, {len(comp.faces)} faces")
        print(f"    Center: {comp.centroid}")
        print(f"    Bounds: {comp.bounds}")
    
    return components


def split_by_plane(mesh, axis='x', position=None):
    """
    Split mesh by a cutting plane.
    
    Args:
        mesh: trimesh mesh object
        axis: 'x', 'y', or 'z' - axis perpendicular to cutting plane
        position: position along axis to cut (default: center)
    """
    import trimesh
    
    axis_map = {'x': 0, 'y': 1, 'z': 2}
    axis_idx = axis_map[axis.lower()]
    
    if position is None:
        position = mesh.centroid[axis_idx]
    
    print(f"Splitting along {axis}-axis at position {position}")
    
    # Create plane normal
    plane_normal = np.zeros(3)
    plane_normal[axis_idx] = 1.0
    plane_origin = np.zeros(3)
    plane_origin[axis_idx] = position
    
    # Split the mesh
    try:
        result = mesh.slice_plane(plane_origin, plane_normal, cap=True)
        result_neg = mesh.slice_plane(plane_origin, -plane_normal, cap=True)
        
        components = []
        if result is not None and len(result.vertices) > 0:
            components.append(result)
        if result_neg is not None and len(result_neg.vertices) > 0:
            components.append(result_neg)
            
        if len(components) < 2:
            print("Warning: Plane split didn't produce two parts. Trying alternative method...")
            return split_by_vertex_position(mesh, axis_idx, position)
            
        return components
    except Exception as e:
        print(f"Plane slicing failed: {e}")
        print("Falling back to vertex-based splitting...")
        return split_by_vertex_position(mesh, axis_idx, position)


def split_by_vertex_position(mesh, axis_idx, position):
    """
    Split mesh by selecting faces based on their centroid position.
    This is a fallback when plane slicing doesn't work well.
    """
    import trimesh
    
    # Calculate face centroids
    face_centroids = mesh.triangles_center
    
    # Split faces by position
    positive_faces = face_centroids[:, axis_idx] >= position
    negative_faces = ~positive_faces
    
    components = []
    
    for mask, name in [(positive_faces, 'positive'), (negative_faces, 'negative')]:
        if mask.sum() > 0:
            # Get faces for this side
            faces = mesh.faces[mask]
            
            # Get unique vertices used by these faces
            unique_verts = np.unique(faces.flatten())
            
            # Create vertex mapping
            vert_map = {old: new for new, old in enumerate(unique_verts)}
            
            # Remap faces
            new_faces = np.array([[vert_map[v] for v in face] for face in faces])
            new_vertices = mesh.vertices[unique_verts]
            
            # Create new mesh
            new_mesh = trimesh.Trimesh(vertices=new_vertices, faces=new_faces)
            components.append(new_mesh)
            print(f"  {name} side: {len(new_vertices)} vertices, {len(new_faces)} faces")
    
    return components


def analyze_mesh(mesh):
    """Analyze mesh to suggest best splitting method."""
    import trimesh
    
    print("\n=== Mesh Analysis ===")
    
    # Check for connected components
    components = mesh.split(only_watertight=False)
    print(f"Connected components: {len(components)}")
    
    if len(components) > 1:
        print("✓ Mesh has multiple disconnected parts - use --method connected_components")
    else:
        print("Mesh is a single connected component - analyze symmetry...")
        
        # Analyze symmetry
        bounds = mesh.bounds
        extents = bounds[1] - bounds[0]
        center = mesh.centroid
        
        print(f"Extents (X, Y, Z): {extents}")
        print(f"Center: {center}")
        
        # Suggest splitting axis based on extents
        if extents[0] > max(extents[1], extents[2]):
            print("→ Widest along X-axis - try: --method plane --axis x")
        elif extents[1] > max(extents[0], extents[2]):
            print("→ Widest along Y-axis - try: --method plane --axis y")
        else:
            print("→ Widest along Z-axis - try: --method plane --axis z")
    
    return len(components) > 1


def save_components(components, output_dir, base_name):
    """Save mesh components to separate STL files."""
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    saved_files = []
    
    for i, comp in enumerate(components):
        if i == 0:
            suffix = "left"
        elif i == 1:
            suffix = "right"
        else:
            suffix = f"part{i}"
            
        output_path = output_dir / f"{base_name}_{suffix}.stl"
        comp.export(str(output_path))
        saved_files.append(output_path)
        print(f"Saved: {output_path}")
    
    return saved_files


def generate_mujoco_xml(stl_files, output_path):
    """Generate example MuJoCo XML configuration for the split gripper parts."""
    
    xml_template = '''<?xml version="1.0" encoding="utf-8"?>
<mujoco model="gripper">
    <compiler angle="radian" meshdir="meshes"/>
    
    <asset>
        <!-- Gripper mesh assets -->
{mesh_assets}
    </asset>
    
    <worldbody>
        <!-- Ground plane -->
        <geom type="plane" size="1 1 0.1" rgba="0.8 0.8 0.8 1"/>
        
        <!-- Robot arm base (simplified) -->
        <body name="arm_base" pos="0 0 0.5">
            <geom type="cylinder" size="0.05 0.2" rgba="0.5 0.5 0.5 1"/>
            
            <!-- Gripper base -->
            <body name="gripper_base" pos="0 0 0.25">
                <geom type="box" size="0.02 0.05 0.02" rgba="0.3 0.3 0.3 1"/>
                
                <!-- Left gripper finger -->
                <body name="gripper_left" pos="-0.03 0 0">
                    <joint name="finger_left" type="slide" axis="1 0 0" 
                           range="-0.05 0.05" damping="10"/>
{left_geom}
                </body>
                
                <!-- Right gripper finger -->
                <body name="gripper_right" pos="0.03 0 0">
                    <joint name="finger_right" type="slide" axis="1 0 0" 
                           range="-0.05 0.05" damping="10"/>
{right_geom}
                </body>
            </body>
        </body>
    </worldbody>
    
    <actuator>
        <!-- Position control for gripper fingers -->
        <position name="left_finger_ctrl" joint="finger_left" kp="100"/>
        <position name="right_finger_ctrl" joint="finger_right" kp="100"/>
    </actuator>
    
    <equality>
        <!-- Optional: Couple the fingers to move together (symmetric gripper) -->
        <!-- Uncomment to make fingers mirror each other -->
        <!-- <joint joint1="finger_left" joint2="finger_right" polycoef="0 -1 0 0 0"/> -->
    </equality>
</mujoco>
'''
    
    # Generate mesh asset entries
    mesh_assets = []
    geom_entries = {'left': '', 'right': ''}
    
    for stl_file in stl_files:
        name = stl_file.stem
        mesh_assets.append(f'        <mesh name="{name}" file="{stl_file.name}"/>')
        
        if 'left' in name.lower():
            geom_entries['left'] = f'                    <geom type="mesh" mesh="{name}" rgba="0.8 0.2 0.2 1"/>'
        elif 'right' in name.lower():
            geom_entries['right'] = f'                    <geom type="mesh" mesh="{name}" rgba="0.2 0.2 0.8 1"/>'
    
    xml_content = xml_template.format(
        mesh_assets='\n'.join(mesh_assets),
        left_geom=geom_entries['left'],
        right_geom=geom_entries['right']
    )
    
    with open(output_path, 'w') as f:
        f.write(xml_content)
    
    print(f"\nGenerated MuJoCo XML: {output_path}")
    return output_path


def main():
    parser = argparse.ArgumentParser(
        description='Split STL file for MuJoCo gripper simulation',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
    # Analyze mesh to determine best splitting method
    python split_stl.py gripper.stl --analyze
    
    # Split by connected components (when parts are separate)
    python split_stl.py gripper.stl --method connected_components
    
    # Split by plane along X-axis at center
    python split_stl.py gripper.stl --method plane --axis x
    
    # Split by plane at specific position
    python split_stl.py gripper.stl --method plane --axis x --position 0.0
    
    # Generate MuJoCo XML as well
    python split_stl.py gripper.stl --method plane --axis x --generate-xml
        '''
    )
    
    parser.add_argument('input_stl', nargs='?', help='Input STL file path')
    parser.add_argument('--analyze', action='store_true', 
                        help='Analyze mesh and suggest splitting method')
    parser.add_argument('--method', choices=['connected_components', 'plane'], 
                        default='connected_components',
                        help='Splitting method (default: connected_components)')
    parser.add_argument('--axis', choices=['x', 'y', 'z'], default='x',
                        help='Axis for plane splitting (default: x)')
    parser.add_argument('--position', type=float, default=None,
                        help='Position along axis for plane split (default: center)')
    parser.add_argument('--output-dir', '-o', default=None,
                        help='Output directory (default: same as input)')
    parser.add_argument('--generate-xml', action='store_true',
                        help='Generate example MuJoCo XML configuration')
    parser.add_argument('--demo', action='store_true',
                        help='Create a demo gripper STL and split it')
    
    args = parser.parse_args()
    
    # Install dependencies
    install_dependencies()
    import trimesh
    
    # Demo mode - create sample gripper
    if args.demo:
        print("Creating demo gripper mesh...")
        stl_path = create_demo_gripper()
        args.input_stl = str(stl_path)
    
    if not args.input_stl:
        parser.print_help()
        print("\nError: Please provide an input STL file or use --demo")
        return
    
    input_path = Path(args.input_stl)
    if not input_path.exists():
        print(f"Error: File not found: {input_path}")
        return
    
    # Load mesh
    mesh = load_mesh(input_path)
    
    # Analyze mode
    if args.analyze:
        analyze_mesh(mesh)
        return
    
    # Split mesh
    print(f"\nSplitting using method: {args.method}")
    
    if args.method == 'connected_components':
        components = split_by_connected_components(mesh)
    else:  # plane
        components = split_by_plane(mesh, args.axis, args.position)
    
    if len(components) < 2:
        print("\nWarning: Could not split into 2 parts. Try a different method or parameters.")
        if args.method == 'connected_components':
            print("Suggestion: Try --method plane --axis x (or y/z)")
        return
    
    # Save components
    output_dir = Path(args.output_dir) if args.output_dir else input_path.parent / "meshes"
    base_name = input_path.stem
    
    saved_files = save_components(components, output_dir, base_name)
    
    # Generate MuJoCo XML if requested
    if args.generate_xml:
        xml_path = input_path.parent / f"{base_name}_mujoco.xml"
        generate_mujoco_xml(saved_files, xml_path)
    
    print("\n✓ Done! Your gripper parts are ready for MuJoCo simulation.")
    print("\nNext steps:")
    print("1. Check the generated STL files in:", output_dir)
    print("2. Adjust positions and joint parameters in your MuJoCo XML")
    print("3. You may need to adjust mesh scale or origin based on your robot model")


def create_demo_gripper():
    """Create a demo gripper mesh for testing."""
    import trimesh
    
    # Create two finger-like boxes
    left_finger = trimesh.creation.box(extents=[0.02, 0.01, 0.08])
    right_finger = trimesh.creation.box(extents=[0.02, 0.01, 0.08])
    
    # Position them apart
    left_finger.apply_translation([-0.03, 0, 0])
    right_finger.apply_translation([0.03, 0, 0])
    
    # Combine into one mesh
    combined = trimesh.util.concatenate([left_finger, right_finger])
    
    # Save
    output_path = Path('/workspace/demo_gripper.stl')
    combined.export(str(output_path))
    print(f"Created demo gripper: {output_path}")
    
    return output_path


if __name__ == '__main__':
    main()
