#!/usr/bin/env python3
"""
Split a gripper STL file into two separate parts for MuJoCo simulation.

This script provides multiple methods to split a gripper:
1. By connected components (if fingers are separate meshes)
2. By X-axis position (left/right split)
3. By Y-axis position (front/back split)
4. By Z-axis position (top/bottom split)

Usage:
    python split_gripper_stl.py input.stl --method components
    python split_gripper_stl.py input.stl --method x --threshold 0.0
    python split_gripper_stl.py input.stl --method y --threshold 0.0
"""

import argparse
import numpy as np
from pathlib import Path

try:
    import trimesh
except ImportError:
    print("Installing required package: trimesh...")
    import subprocess
    subprocess.check_call(["pip", "install", "trimesh", "networkx"])
    import trimesh

try:
    import networkx
    HAS_GRAPH_ENGINE = True
except ImportError:
    HAS_GRAPH_ENGINE = False


def split_by_connected_components(mesh: trimesh.Trimesh) -> list:
    """
    Split mesh into connected components.
    Works best when gripper fingers are separate meshes combined into one file.
    """
    if not HAS_GRAPH_ENGINE:
        print("Error: 'networkx' package required for component splitting.")
        print("Install it with: pip install networkx")
        print("Or use axis-based splitting: --method x (or y, z)")
        return [mesh]
    
    try:
        components = mesh.split(only_watertight=False)
        print(f"Found {len(components)} connected component(s)")
        return components
    except Exception as e:
        print(f"Error splitting by components: {e}")
        print("Try axis-based splitting instead: --method x (or y, z)")
        return [mesh]


def split_by_axis(mesh: trimesh.Trimesh, axis: str = 'x', threshold: float = None) -> list:
    """
    Split mesh by position along an axis.
    
    Args:
        mesh: Input mesh
        axis: 'x', 'y', or 'z'
        threshold: Split point along axis. If None, uses centroid.
    
    Returns:
        List of two meshes (or one if all triangles are on one side)
    """
    axis_idx = {'x': 0, 'y': 1, 'z': 2}[axis.lower()]
    
    # Get triangle centroids
    triangle_centroids = mesh.triangles_center
    
    # Use mesh centroid if no threshold specified
    if threshold is None:
        threshold = mesh.centroid[axis_idx]
        print(f"Using mesh centroid as threshold: {threshold:.4f}")
    
    # Split triangles based on their centroid position
    mask_left = triangle_centroids[:, axis_idx] < threshold
    mask_right = ~mask_left
    
    meshes = []
    
    # Create left/lower mesh
    if np.any(mask_left):
        faces_left = mesh.faces[mask_left]
        mesh_left = trimesh.Trimesh(vertices=mesh.vertices, faces=faces_left)
        mesh_left.remove_unreferenced_vertices()
        meshes.append(mesh_left)
        print(f"Part 1 ({axis}<{threshold:.4f}): {len(faces_left)} faces")
    
    # Create right/upper mesh
    if np.any(mask_right):
        faces_right = mesh.faces[mask_right]
        mesh_right = trimesh.Trimesh(vertices=mesh.vertices, faces=faces_right)
        mesh_right.remove_unreferenced_vertices()
        meshes.append(mesh_right)
        print(f"Part 2 ({axis}>={threshold:.4f}): {len(faces_right)} faces")
    
    return meshes


def split_by_plane(mesh: trimesh.Trimesh, plane_origin: np.ndarray, plane_normal: np.ndarray) -> list:
    """
    Split mesh by an arbitrary plane.
    
    Args:
        mesh: Input mesh
        plane_origin: Point on the plane
        plane_normal: Normal vector of the plane
    
    Returns:
        List of two meshes
    """
    # Calculate signed distance of each triangle centroid to the plane
    triangle_centroids = mesh.triangles_center
    distances = np.dot(triangle_centroids - plane_origin, plane_normal)
    
    mask_positive = distances >= 0
    mask_negative = ~mask_positive
    
    meshes = []
    
    if np.any(mask_negative):
        faces_neg = mesh.faces[mask_negative]
        mesh_neg = trimesh.Trimesh(vertices=mesh.vertices, faces=faces_neg)
        mesh_neg.remove_unreferenced_vertices()
        meshes.append(mesh_neg)
        print(f"Part 1 (negative side): {len(faces_neg)} faces")
    
    if np.any(mask_positive):
        faces_pos = mesh.faces[mask_positive]
        mesh_pos = trimesh.Trimesh(vertices=mesh.vertices, faces=faces_pos)
        mesh_pos.remove_unreferenced_vertices()
        meshes.append(mesh_pos)
        print(f"Part 2 (positive side): {len(faces_pos)} faces")
    
    return meshes


def analyze_mesh(mesh: trimesh.Trimesh):
    """Print useful information about the mesh."""
    print("\n=== Mesh Analysis ===")
    print(f"Vertices: {len(mesh.vertices)}")
    print(f"Faces: {len(mesh.faces)}")
    print(f"Bounds:")
    print(f"  X: {mesh.bounds[0][0]:.4f} to {mesh.bounds[1][0]:.4f} (extent: {mesh.extents[0]:.4f})")
    print(f"  Y: {mesh.bounds[0][1]:.4f} to {mesh.bounds[1][1]:.4f} (extent: {mesh.extents[1]:.4f})")
    print(f"  Z: {mesh.bounds[0][2]:.4f} to {mesh.bounds[1][2]:.4f} (extent: {mesh.extents[2]:.4f})")
    print(f"Centroid: [{mesh.centroid[0]:.4f}, {mesh.centroid[1]:.4f}, {mesh.centroid[2]:.4f}]")
    
    # Check for connected components (requires networkx or scipy)
    if HAS_GRAPH_ENGINE:
        try:
            components = mesh.split(only_watertight=False)
            print(f"Connected components: {len(components)}")
            
            if len(components) > 1:
                print("\nComponent sizes:")
                for i, comp in enumerate(components):
                    print(f"  Component {i+1}: {len(comp.faces)} faces")
        except Exception as e:
            print(f"Could not analyze components: {e}")
    else:
        print("Connected components: (install 'networkx' to detect)")
    
    # Suggest split axis based on extents
    axes = ['x', 'y', 'z']
    max_extent_idx = np.argmax(mesh.extents)
    print(f"\nSuggested split axis: '{axes[max_extent_idx]}' (largest extent)")
    print(f"  Command: python3 split_gripper_stl.py your_file.stl --method {axes[max_extent_idx]}")
    
    print("=" * 25 + "\n")


def save_meshes(meshes: list, input_path: Path, output_dir: Path = None, 
                names: list = None):
    """Save split meshes to separate STL files."""
    if output_dir is None:
        output_dir = input_path.parent
    
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    base_name = input_path.stem
    
    if names is None:
        names = [f"left", f"right"] if len(meshes) == 2 else [f"part{i+1}" for i in range(len(meshes))]
    
    saved_files = []
    for i, (mesh, name) in enumerate(zip(meshes, names)):
        output_path = output_dir / f"{base_name}_{name}.stl"
        mesh.export(output_path)
        print(f"Saved: {output_path}")
        saved_files.append(output_path)
    
    return saved_files


def main():
    parser = argparse.ArgumentParser(
        description="Split a gripper STL file into separate parts for MuJoCo simulation"
    )
    parser.add_argument("input", type=str, help="Input STL file path")
    parser.add_argument(
        "--method", 
        type=str, 
        choices=["components", "x", "y", "z", "plane"],
        default="components",
        help="Split method: 'components' (auto-detect), 'x'/'y'/'z' (axis split), 'plane' (custom plane)"
    )
    parser.add_argument(
        "--threshold", 
        type=float, 
        default=None,
        help="Split threshold along axis (default: mesh centroid)"
    )
    parser.add_argument(
        "--plane-origin",
        type=float,
        nargs=3,
        default=[0, 0, 0],
        help="Plane origin for plane split (x y z)"
    )
    parser.add_argument(
        "--plane-normal",
        type=float,
        nargs=3,
        default=[1, 0, 0],
        help="Plane normal for plane split (x y z)"
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default=None,
        help="Output directory (default: same as input)"
    )
    parser.add_argument(
        "--analyze-only",
        action="store_true",
        help="Only analyze the mesh, don't split"
    )
    parser.add_argument(
        "--names",
        type=str,
        nargs="+",
        default=None,
        help="Custom names for output files (e.g., --names finger_left finger_right)"
    )
    
    args = parser.parse_args()
    
    input_path = Path(args.input)
    if not input_path.exists():
        print(f"Error: File not found: {input_path}")
        return 1
    
    print(f"Loading: {input_path}")
    mesh = trimesh.load(input_path)
    
    # Handle scene vs mesh
    if isinstance(mesh, trimesh.Scene):
        print("Input is a scene, combining all geometries...")
        mesh = mesh.dump(concatenate=True)
    
    analyze_mesh(mesh)
    
    if args.analyze_only:
        return 0
    
    # Split based on method
    if args.method == "components":
        meshes = split_by_connected_components(mesh)
        if len(meshes) < 2:
            print("\nOnly 1 component found. Try splitting by axis instead:")
            print("  python split_gripper_stl.py input.stl --method x")
            print("  python split_gripper_stl.py input.stl --method y")
            return 1
    elif args.method in ["x", "y", "z"]:
        meshes = split_by_axis(mesh, axis=args.method, threshold=args.threshold)
    elif args.method == "plane":
        origin = np.array(args.plane_origin)
        normal = np.array(args.plane_normal)
        normal = normal / np.linalg.norm(normal)  # Normalize
        meshes = split_by_plane(mesh, origin, normal)
    
    if len(meshes) < 2:
        print("Error: Could not split mesh into 2 parts")
        return 1
    
    # Save output
    output_dir = Path(args.output_dir) if args.output_dir else None
    saved_files = save_meshes(meshes, input_path, output_dir, args.names)
    
    print(f"\n✓ Successfully split into {len(meshes)} parts!")
    print("\nFor MuJoCo, add these meshes to your XML:")
    print("-" * 50)
    for f in saved_files:
        print(f'  <mesh name="{f.stem}" file="{f.name}"/>')
    print("-" * 50)
    
    return 0


if __name__ == "__main__":
    exit(main())
