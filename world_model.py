import pyhop
import time


class WorldModel:
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
            "init_servo_values": [1500, 1500, 1500, 1500, 1500, 1500]}  # TODO: Move 2 gremlin world model with init
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
        self.current_world_model.real_time_clock_period_seconds = {"arm": 0.5}
        self.current_world_model.init_delay_seconds = {"arm": 5}
        self.current_world_model.plans = []

        # Perception
        self.current_world_model.perception = {"MAX_FEATURES": 900,  # 900
                                               "MIN_MATCHES": 30,  # 15
                                               "GOOD_MATCH_PERCENT": 0.3,  # 0.3
                                               "FLASH_EVERY_FRAMES": 40.0,
                                               "MIN_DESCRIPTOR_DISTANCE_SUM": 10000,
                                               "use_flann": True,
                                               "FLANN_INDEX_LSH": 6,
                                               "regressor_qr_to_arm_xyz":
                                                   {"file_path": "modelsQr/pixels_qr_RANSACRegressor_xyz.sav"},
                                               "class_logo": {"file_path": "picsQr/logoTarget.png"},
                                               "model_reference": {"file_path": "picsQr/modelTarget.png"},
                                               "input_video": {"file_path": "picsQr/vids/good8.mp4"},  # TODO: use
                                               "output_video": {"file_path": "perception.avi"},  # TODO: use
                                               "video_frames_per_second": 15,  # 15
                                               "arm_xyz_offset": [0.0, 0.0, 0.0],
                                               "use_local_camera": True,
                                               "camera_frame_width": 1920,
                                               "camera_frame_height": 1080,
                                               "auto_focus": True,
                                               "send_requests": True,
                                               "verbose": False,
                                               "percept_frames": 5,
                                               "write_video": False,
                                               "display_output_frames": True,
                                               "local_camera_id": 0}

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

    # "local_camera_id": 0
    print('world_model.current_world_model.location["local_camera_id"]: {}'
          .format(world_model.current_world_model.perception["local_camera_id"]))
