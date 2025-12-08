import argparse

import grasp_box.scenario
from grasp_box.dependencies import Dependencies
from grasp_box.launch_options import *

dependencies: Dependencies
ros2_enabled: bool = False


def step(time: float, dt: float):
    """Main simulation step function called by the launcher.
    
    Args:
        time: Current simulation time in seconds.
        dt: Time step duration.
    """
    dependencies.scenario.step(time, dt)
    
    dependencies.target_left_mocap_controller.step(dt)
    dependencies.target_right_mocap_controller.step(dt)
    
    dependencies.cart_pos_ctrl.step(dt)
    dependencies.robot_body_inverse_dynamics_ctrl.step(dt)
    dependencies.body_pos_ctrl.step(dt)
    dependencies.body_up_pos_ctrl.step(dt)
    dependencies.torso_pos_ctrl.step(dt)
    dependencies.torso_up_pos_ctrl.step(dt)
    dependencies.torso_down_pos_ctrl.step(dt)
    dependencies.arm_left_tree_pos_ctrl.step(dt)
    dependencies.arm_left_pos_ctrl.step(dt)
    dependencies.arm_right_tree_pos_ctrl.step(dt)
    dependencies.arm_right_pos_ctrl.step(dt)
    dependencies.hand_left_pos_ctrl.step(dt)
    dependencies.lh_tf_pos_ctrl.step(dt)
    dependencies.lh_pf_pos_ctrl.step(dt)
    dependencies.lh_mf_pos_ctrl.step(dt)
    dependencies.lh_rf_pos_ctrl.step(dt)
    dependencies.lh_lf_pos_ctrl.step(dt)
    dependencies.hand_right_pos_ctrl.step(dt)
    dependencies.rh_tf_pos_ctrl.step(dt)
    dependencies.rh_pf_pos_ctrl.step(dt)
    dependencies.rh_mf_pos_ctrl.step(dt)
    dependencies.rh_rf_pos_ctrl.step(dt)
    dependencies.rh_lf_pos_ctrl.step(dt)
    
    # dependencies.cart_impedance_ctrl.step(dt)
    dependencies.body_impedance_ctrl.step(dt)
    dependencies.body_up_impedance_ctrl.step(dt)
    dependencies.torso_impedance_ctrl.step(dt)
    dependencies.torso_up_impedance_ctrl.step(dt)
    dependencies.torso_down_impedance_ctrl.step(dt)
    dependencies.arm_left_tree_impedance_ctrl.step(dt)
    dependencies.arm_left_impedance_ctrl.step(dt)
    dependencies.arm_right_tree_impedance_ctrl.step(dt)
    dependencies.arm_right_impedance_ctrl.step(dt)
    dependencies.hand_left_impedance_ctrl.step(dt)
    dependencies.lh_tf_impedance_ctrl.step(dt)
    dependencies.lh_pf_impedance_ctrl.step(dt)
    dependencies.lh_mf_impedance_ctrl.step(dt)
    dependencies.lh_rf_impedance_ctrl.step(dt)
    dependencies.lh_lf_impedance_ctrl.step(dt)
    dependencies.hand_right_impedance_ctrl.step(dt)
    dependencies.rh_tf_impedance_ctrl.step(dt)
    dependencies.rh_pf_impedance_ctrl.step(dt)
    dependencies.rh_mf_impedance_ctrl.step(dt)
    dependencies.rh_rf_impedance_ctrl.step(dt)
    dependencies.rh_lf_impedance_ctrl.step(dt)

    dependencies.camera_controller.step(dt)
    dependencies.joints_limits_controller.step(dt)
    dependencies.fire_equality_controller.step(dt)
    
    # === ROS2 Integration (only if enabled) ===
    if ros2_enabled and dependencies.mujoco_ROS2_bridge is not None:
        import rclpy
        # Step the ROS2 bridge to publish data at the configured rate
        dependencies.mujoco_ROS2_bridge.step(time)
        
        # Process any pending ROS2 callbacks (subscriptions, services, etc.)
        # timeout_sec=0 means non-blocking - returns immediately if no work
        rclpy.spin_once(dependencies.mujoco_ROS2_bridge, timeout_sec=0)
    
    return


def parse_arguments():
    parser = argparse.ArgumentParser(description="Run simulation with specified launch options.")

    parser.add_argument("--scenario_name", type=str, default=SCENARIO_PRESET_KITCHEN_OVEN, choices=[
        SCENARIO_PRESET_KITCHEN_OVEN, SCENARIO_2_BOX_45D, SCENARIO_2_1_BOX_45DF, SCENARIO_2_2_BOX_0D, SCENARIO_2_3_BOX_90D
    ], help="Hands grasp position scenario to run.")

    parser.add_argument("--box_name", type=str, default=R307, choices=[
        REAL_BOX, WHITE_BOX, OPEN_BOX, R307
    ], help="Box to use in the scenario.")

    parser.add_argument("--active_camera_controller", action='store_true', help="Enable camera controller.")
    parser.add_argument("--no_active_camera_controller", action='store_false', dest="active_camera_controller")
    parser.set_defaults(active_camera_controller=True)

    parser.add_argument("--render_video", action='store_true', help="Render video output.")
    parser.set_defaults(render_video=False)
    parser.add_argument("--headless_mode", action='store_true', help="Run in headless mode.")
    parser.set_defaults(headless_mode=False)
    parser.add_argument("--finish_scenario", action='store_true', help="Auto-finish scenario.")
    parser.set_defaults(finish_scenario=False)
   
    parser.add_argument("--collect_data", action='store_true', help="Enable data collection.")
    parser.add_argument("--no_collect_data", action='store_false', dest="collect_data")
    parser.set_defaults(collect_data=False)

    parser.add_argument("--camera_scenario", type=str, default=THIRD_PERSON_POV, choices=[
        LEFT_HAND_POV, RIGHT_HAND_POV, THIRD_PERSON_POV, FIRST_PERSON_POV
    ], help="Camera scenario mode.")

    parser.add_argument("--robot_drive_scenario", type=str, default=ROT180_MOVE_FLAG21, choices=[
        ROTATE_RIGHT_AND_SIDE_DRIVE_SCENARIO, ROTATE_LEFT_AND_FORWARD_DRIVE_SCENARIO
    ], help="Robot driving behavior.")

    parser.add_argument("--video_speed", type=float, default=1.0, help="Speed multiplier for video playback.")
    parser.add_argument("--video_framerate", type=int, default=60, help="Framerate for the video.")
    parser.add_argument("--video_partition_by", type=float, default=None, help="Time to partition video by (in seconds).")

    parser.add_argument("--publish_ros2", action='store_true', help="Enable ROS2 data publishing.")
    parser.add_argument("--no_publish_ros2", action='store_false', dest="publish_ros2")
    parser.set_defaults(publish_ros2=False)

    return parser.parse_args()


if __name__ == "__main__":
    args = parse_arguments()

    # === Initialize ROS2 only if publishing is enabled ===
    ros2_enabled = args.publish_ros2
    
    # When ROS2 is enabled, automatically enable required flags
    if ros2_enabled:
        args.render_video = True
        args.finish_scenario = True
        args.headless_mode = True
        
        import rclpy
        rclpy.init()

    # movement_mode_robot_joints = POSITIONAL_JOINT_CONTROL
    movement_mode_robot_joints = IMPEDANCE_ACTUATOR_CONTROL
    # movement_mode_robot_joints = INVERSE_DYNAMICS_ACTUATOR_CONTROL

    launch_options = LaunchOptions(
        scenario_name=args.scenario_name,
        box_name=args.box_name,
        joint_control=movement_mode_robot_joints,
        active_camera_controller=args.active_camera_controller,
        render_video=args.render_video,
        headless_mode=args.headless_mode,
        finish_scenario=args.finish_scenario,
        collect_data=args.collect_data,
        camera_scenario=args.camera_scenario,
        robot_drive_scenario=args.robot_drive_scenario,
        video_speed=args.video_speed,
        video_framerate=args.video_framerate,
        video_partition_by=args.video_partition_by,
        publish_ros2=ros2_enabled,
    )

    try:
        dependencies = Dependencies(
            step=step,
            launch_options=launch_options
        )
        grasp_box.scenario.setup(dependencies, launch_options)
        
        if launch_options.headless_mode:
            dependencies.launcher.start_headless()
        else:
            dependencies.launcher.start(0)
    finally:
        # === Cleanup ROS2 (only if it was enabled) ===
        if ros2_enabled:
            import rclpy
            if dependencies is not None and dependencies.mujoco_ROS2_bridge is not None:
                dependencies.mujoco_ROS2_bridge.destroy_node()
            rclpy.shutdown()
