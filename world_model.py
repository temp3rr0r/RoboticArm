import pyhop
import time
import json
import os


class WorldModel:  # TODO: Move 2 gremlin world model with init
    """
    Stores and updates current & past world models, instances of a Pyhop State class.
    """

    def __init__(self):
        self.current_world_model = pyhop.State("current_world_model")
        self.current_world_model.tick = 0
        self.current_world_model.max_ticks = 100
        self.current_world_model.timestamp = time.time()

        self.current_world_model.location = {
            "target_object": "table",
            "servo_values": [1500, 1500, 1500, 1500, 1500, 1500],
            "init_servo_values": [1500, 1500, 1500, 1500, 1500, 1500]}
        self.current_world_model.xyz = {"target_object": [-30, -30, 0],
                                        "container": [-0.1, 24.0, 12],
                                        "end_effector": [-0.1, 24.0, 12]
                                        }
        self.current_world_model.size = {"object_side_length": 4.0}
        self.current_world_model.min_bounds = {"xyz": [-25, -25, -25], "object_side_length": 0.5}
        self.current_world_model.max_bounds = {"xyz": [25, 25, 25], "object_side_length": 6.0}
        self.current_world_model.threshold = {"grabbing_distance": 4.5}  # cm
        self.current_world_model.distance = {"distance_to_gripper": 11.2}  # cm
        self.current_world_model.grabbed = {"target_object": False}
        self.current_world_model.initialized = {"arm": False}
        self.current_world_model.url = {"arm": "ESP_02662E"}
        self.current_world_model.init_delay_seconds = {"arm": 5}
        self.current_world_model.real_time_clock_period_seconds = {"arm": 0.5}

        # Control
        if os.path.isfile('json/control.json'):
            with open('json/control.json') as f:
                self.current_world_model.control = json.load(f)

        # Planner
        if os.path.isfile('json/planner.json'):
            with open('json/planner.json') as f:
                self.current_world_model.planner = json.load(f)

        # Goals
        self.current_world_model.goals = [tuple(self.current_world_model.planner["goals"])]

        # Perception
        if os.path.isfile('json/perception.json'):
            with open('json/perception.json') as f:
                self.current_world_model.perception = json.load(f)

        self.world_model_history = []

    def update_tick(self):
        """
        Updates the tick count and the current timestamp.
        :return: The current tick.
        """
        self.current_world_model.tick += 1
        self.current_world_model.timestamp = time.time()
        return self.current_world_model.tick


if __name__ == '__main__':

    # Sequence for testing
    world_model = WorldModel()
    print("world_model.current_world_model.perception: {}".format(world_model.current_world_model.perception))
    print('world_model.current_world_model.perception["MAX_FEATURES"]: {}'
          .format(world_model.current_world_model.perception["MAX_FEATURES"]))
    print('world_model.current_world_model.perception["MIN_MATCHES"]: {}'
          .format(world_model.current_world_model.perception["MIN_MATCHES"]))
    print('world_model.current_world_model.perception["GOOD_MATCH_PERCENT"]: {}'
          .format(world_model.current_world_model.perception["GOOD_MATCH_PERCENT"]))
    print('world_model.current_world_model.perception["FLASH_EVERY_FRAMES"]: {}'
          .format(world_model.current_world_model.perception["FLASH_EVERY_FRAMES"]))
    print('world_model.current_world_model.perception["MIN_DESCRIPTOR_DISTANCE_SUM"]: {}'
          .format(world_model.current_world_model.perception["MIN_DESCRIPTOR_DISTANCE_SUM"]))
    print('world_model.current_world_model.perception["use_flann"]: {}'
          .format(world_model.current_world_model.perception["use_flann"]))
    print('world_model.current_world_model.perception["FLANN_INDEX_LSH"]: {}'
          .format(world_model.current_world_model.perception["FLANN_INDEX_LSH"]))
    print('world_model.current_world_model.perception["regressor_qr_to_arm_xyz"]: {}'
          .format(world_model.current_world_model.perception["regressor_qr_to_arm_xyz"]["file_path"]))
    print('world_model.current_world_model.perception["class_logo"]: {}'
          .format(world_model.current_world_model.perception["class_logo"]["file_path"]))
    print('world_model.current_world_model.perception["model_reference"]: {}'
          .format(world_model.current_world_model.perception["model_reference"]["file_path"]))
    print('world_model.current_world_model.perception["video_frames_per_second"]: {}'
          .format(world_model.current_world_model.perception["video_frames_per_second"]))
    print('world_model.current_world_model.perception["arm_xyz_offset"]: {}'
          .format(world_model.current_world_model.perception["arm_xyz_offset"]))
    print('world_model.current_world_model.perception["use_local_camera"]: {}'
          .format(world_model.current_world_model.perception["use_local_camera"]))
    print('world_model.current_world_model.perception["camera_frame_width"]: {}'
          .format(world_model.current_world_model.perception["camera_frame_width"]))
    print('world_model.current_world_model.perception["camera_frame_height"]: {}'
          .format(world_model.current_world_model.perception["camera_frame_height"]))
    print('world_model.current_world_model.perception["auto_focus"]: {}'
          .format(world_model.current_world_model.perception["auto_focus"]))
    print('world_model.current_world_model.perception["send_requests"]: {}'
          .format(world_model.current_world_model.perception["send_requests"]))
    print('world_model.current_world_model.perception["verbose"]: {}'
          .format(world_model.current_world_model.perception["verbose"]))
    print('world_model.current_world_model.perception["percept_frames"]: {}'
          .format(world_model.current_world_model.perception["percept_frames"]))
    print('world_model.current_world_model.perception["write_video"]: {}'
          .format(world_model.current_world_model.perception["write_video"]))
    print('world_model.current_world_model.perception["display_output_frames"]: {}'
          .format(world_model.current_world_model.perception["display_output_frames"]))
    print('world_model.current_world_model.url["arm"]: {}'
          .format(world_model.current_world_model.url["arm"]))
    print('world_model.current_world_model.location["init_servo_values"]: {}'
          .format(world_model.current_world_model.location["init_servo_values"]))
    print('world_model.current_world_model.location["local_camera_id"]: {}'
          .format(world_model.current_world_model.perception["local_camera_id"]))

    print("world_model.current_world_model.control: {}".format(world_model.current_world_model.control))
    print('world_model.current_world_model.control["closed_hand_distance_ratio"]: {}'
          .format(world_model.current_world_model.control["closed_hand_distance_ratio"]))
    print('world_model.current_world_model.control["opened_hand_distance_ratio"]: {}'
          .format(world_model.current_world_model.control["opened_hand_distance_ratio"]))
    print('world_model.current_world_model.control["base_put_url"]: {}'
          .format(world_model.current_world_model.control["base_put_url"]))
    print('world_model.current_world_model.control["send_requests"]: {}'
          .format(world_model.current_world_model.control["send_requests"]))
    print('world_model.current_world_model.control["detect_last_position"]: {}'
          .format(world_model.current_world_model.control["detect_last_position"]))
    print('world_model.current_world_model.control["show_plots"]: {}'
          .format(world_model.current_world_model.control["show_plots"]))
    print('world_model.current_world_model.control["cm_to_servo_polynomial_fitter"]["file_path"]: {}'
          .format(world_model.current_world_model.control["cm_to_servo_polynomial_fitter"]["file_path"]))
    print('world_model.current_world_model.control["scale"]: {}'
          .format(world_model.current_world_model.control["scale"]))
    print('world_model.current_world_model.control["servo_count"]: {}'
          .format(world_model.current_world_model.control["servo_count"]))
    print('world_model.current_world_model.control["command_delay"]: {}'
          .format(world_model.current_world_model.control["command_delay"]))
    print('world_model.current_world_model.control["center_init"]: {}'
          .format(world_model.current_world_model.control["center_init"]))
    print('world_model.current_world_model.control["angle_degree_limit"]: {}'
          .format(world_model.current_world_model.control["angle_degree_limit"]))
    print('world_model.current_world_model.control["trajectory_steps"]: {}'
          .format(world_model.current_world_model.control["trajectory_steps"]))
    print('world_model.current_world_model.control["current_servo_monotony"]: {}'
          .format(world_model.current_world_model.control["current_servo_monotony"]))
    print('world_model.current_world_model.control["active_links_mask"]: {}'
          .format(world_model.current_world_model.control["active_links_mask"]))
    print('world_model.current_world_model.control["min_steps"]: {}'
          .format(world_model.current_world_model.control["min_steps"]))
    print('world_model.current_world_model.control["max_steps"]: {}'
          .format(world_model.current_world_model.control["max_steps"]))
    print('world_model.current_world_model.control["rotating_gripper_servo"]: {}'
          .format(world_model.current_world_model.control["rotating_gripper_servo"]))
    print('world_model.current_world_model.control["gripping_gripper_servo"]: {}'
          .format(world_model.current_world_model.control["gripping_gripper_servo"]))
    print('world_model.current_world_model.control["horizontal_gripper_position"]: {}'
          .format(world_model.current_world_model.control["horizontal_gripper_position"]))
    print('world_model.current_world_model.control["init_position"]: {}'
          .format(world_model.current_world_model.control["init_position"]))
    print('world_model.current_world_model.control["container_position"]: {}'
          .format(world_model.current_world_model.control["container_position"]))
    print('world_model.current_world_model.control["chain_name"]: {}'
          .format(world_model.current_world_model.control["chain_name"]))
    print('world_model.current_world_model.control["link_lengths"]: {}'
          .format(world_model.current_world_model.control["link_lengths"]))
    print('world_model.current_world_model.control["link_lengths"]["link6"]: {}'
          .format(world_model.current_world_model.control["link_lengths"]["link6"]))
    print('world_model.current_world_model.control["active_links_mask"]: {}'
          .format(world_model.current_world_model.control["active_links_mask"]))
    print('world_model.current_world_model.control["link_orientations"]: {}'
          .format(world_model.current_world_model.control["link_orientations"]))
    print('world_model.current_world_model.control["joint_rotation_axis"]["link6"]: {}'
          .format(world_model.current_world_model.control["link_orientations"]["link6"]))
    print('world_model.current_world_model.control["joint_rotation_axis"]: {}'
          .format(world_model.current_world_model.control["joint_rotation_axis"]))
    print('world_model.current_world_model.control["joint_rotation_axis"]["link6"]: {}'
          .format(world_model.current_world_model.control["joint_rotation_axis"]["link6"]))

    print("world_model.current_world_model.planner: {}".format(world_model.current_world_model.planner))

    print("world_model.current_world_model.goals: {}".format(world_model.current_world_model.goals))
