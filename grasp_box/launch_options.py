# Launch option constants
SCENARIO_PRESET_KITCHEN_OVEN = "kitchen_oven"
SCENARIO_2_BOX_45D = "2_box_45d"
SCENARIO_2_1_BOX_45DF = "2_1_box_45df"
SCENARIO_2_2_BOX_0D = "2_2_box_0d"
SCENARIO_2_3_BOX_90D = "2_3_box_90d"

REAL_BOX = "real_box"
WHITE_BOX = "white_box"
OPEN_BOX = "open_box"
R307 = "r307"

POSITIONAL_JOINT_CONTROL = "positional"
IMPEDANCE_ACTUATOR_CONTROL = "impedance"
INVERSE_DYNAMICS_ACTUATOR_CONTROL = "inverse_dynamics"

LEFT_HAND_POV = "left_hand"
RIGHT_HAND_POV = "right_hand"
THIRD_PERSON_POV = "third_person"
FIRST_PERSON_POV = "first_person"

ROT180_MOVE_FLAG21 = "rot180_move_flag21"
ROTATE_RIGHT_AND_SIDE_DRIVE_SCENARIO = "rotate_right_side"
ROTATE_LEFT_AND_FORWARD_DRIVE_SCENARIO = "rotate_left_forward"


class LaunchOptions:
    def __init__(
        self,
        scenario_name: str = SCENARIO_PRESET_KITCHEN_OVEN,
        box_name: str = OPEN_BOX,
        joint_control: str = IMPEDANCE_ACTUATOR_CONTROL,
        active_camera_controller: bool = True,
        render_video: bool = False,
        headless_mode: bool = False,
        finish_scenario: bool = False,
        collect_data: bool = False,
        camera_scenario: str = THIRD_PERSON_POV,
        robot_drive_scenario: str = ROT180_MOVE_FLAG21,
        video_speed: float = 1.0,
        video_framerate: int = 60,
        video_partition_by: float = None,
    ):
        self.scenario_name = scenario_name
        self.box_name = box_name
        self.joint_control = joint_control
        self.active_camera_controller = active_camera_controller
        self.render_video = render_video
        self.headless_mode = headless_mode
        self.finish_scenario = finish_scenario
        self.collect_data = collect_data
        self.camera_scenario = camera_scenario
        self.robot_drive_scenario = robot_drive_scenario
        self.video_speed = video_speed
        self.video_framerate = video_framerate
        self.video_partition_by = video_partition_by
