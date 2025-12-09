#!/usr/bin/env python3
"""
Batch runner script for running simulations multiple times with predefined arguments.
"""

import argparse
import subprocess
import sys
from pathlib import Path


def run_simulation(script_path: str, run_number: int, total_runs: int) -> bool:
    """
    Run a single simulation with the predefined arguments.
    
    Args:
        script_path: Path to the simulation script
        run_number: Current run number (1-indexed)
        total_runs: Total number of runs
        
    Returns:
        True if successful, False otherwise
    """
    # Predefined arguments for the simulation
    args = [
        sys.executable,
        script_path,
        "--scenario_name", "SCENARIO_PRESET_KITCHEN_OVEN",
        "--box_name", "R307",
        "--active_camera_controller",
        "--render_video",
        "--headless_mode",
        "--finish_scenario",
        "--collect_data",
        "--robot_drive_scenario", "ROT180_MOVE_FLAG21",
        "--video_speed", "1.0",
        "--video_framerate", "60",
        # Note: video_partition_by is None by default, so we don't pass it
    ]
    
    print(f"\n{'='*60}")
    print(f"Starting simulation run {run_number}/{total_runs}")
    print(f"{'='*60}")
    print(f"Command: {' '.join(args)}")
    print()
    
    try:
        result = subprocess.run(args, check=True)
        print(f"\n✓ Run {run_number}/{total_runs} completed successfully")
        return True
    except subprocess.CalledProcessError as e:
        print(f"\n✗ Run {run_number}/{total_runs} failed with return code {e.returncode}")
        return False
    except FileNotFoundError:
        print(f"\n✗ Error: Script not found at '{script_path}'")
        return False


def main():
    parser = argparse.ArgumentParser(
        description="Run simulation script multiple times with predefined arguments."
    )
    
    parser.add_argument(
        "-n", "--num_runs",
        type=int,
        required=True,
        help="Number of times to run the simulation"
    )
    
    parser.add_argument(
        "-s", "--script",
        type=str,
        default="simulation.py",
        help="Path to the simulation script (default: simulation.py)"
    )
    
    parser.add_argument(
        "--stop_on_failure",
        action="store_true",
        default=False,
        help="Stop execution if any run fails"
    )
    
    args = parser.parse_args()
    
    if args.num_runs <= 0:
        print("Error: Number of runs must be a positive integer")
        sys.exit(1)
    
    # Verify script exists
    script_path = Path(args.script)
    if not script_path.exists():
        print(f"Warning: Script '{args.script}' not found. Proceeding anyway...")
    
    print(f"Starting batch simulation run")
    print(f"Script: {args.script}")
    print(f"Total runs: {args.num_runs}")
    print(f"Stop on failure: {args.stop_on_failure}")
    
    successful_runs = 0
    failed_runs = 0
    
    for i in range(1, args.num_runs + 1):
        success = run_simulation(args.script, i, args.num_runs)
        
        if success:
            successful_runs += 1
        else:
            failed_runs += 1
            if args.stop_on_failure:
                print(f"\nStopping due to failure (--stop_on_failure is enabled)")
                break
    
    # Print summary
    print(f"\n{'='*60}")
    print("BATCH RUN SUMMARY")
    print(f"{'='*60}")
    print(f"Total runs attempted: {successful_runs + failed_runs}")
    print(f"Successful: {successful_runs}")
    print(f"Failed: {failed_runs}")
    print(f"{'='*60}")
    
    # Exit with error code if any runs failed
    if failed_runs > 0:
        sys.exit(1)


if __name__ == "__main__":
    main()
