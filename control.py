from ikpy.chain import Chain
from ikpy.link import URDFLink
import matplotlib.pyplot
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from ikpy import geometry_utils
import requests
import time
from sklearn.externals import joblib


class Control:

    """
    Realization of continuous actions, from world model to desired world.
    """

    def __init__(self, init_world_model):

        # TODO: Do use control_world_model
        self.control_world_model = init_world_model.current_world_model.control

        self.closed_hand_distance_ratio = init_world_model.current_world_model.control["closed_hand_distance_ratio"]
        self.opened_hand_distance_ratio = init_world_model.current_world_model.control["opened_hand_distance_ratio"]
        self.url = init_world_model.current_world_model.url["arm"]
        self.base_put_url = init_world_model.current_world_model.control["base_put_url"]
        self.send_requests = \
            init_world_model.current_world_model.control["send_requests"]  # TODO: make communicator class?
        self.detect_last_position = init_world_model.current_world_model.control["detect_last_position"]
        self.verbose = init_world_model.current_world_model.control["verbose"]
        self.show_plots = init_world_model.current_world_model.control["show_plots"]
        self.cm_to_servo_polynomial_fitter = \
            joblib.load(init_world_model.current_world_model.control["cm_to_servo_polynomial_fitter"]["file_path"])
        self.scale = init_world_model.current_world_model.control["scale"]
        self.servo_count = init_world_model.current_world_model.control["servo_count"]
        self.command_delay = init_world_model.current_world_model.control["command_delay"]  # seconds
        self.center_init = init_world_model.current_world_model.control["center_init"]
        self.angle_degree_limit = init_world_model.current_world_model.control["angle_degree_limit"]  # degrees
        self.trajectory_steps = init_world_model.current_world_model.control["trajectory_steps"]
        self.current_servo_monotony = init_world_model.current_world_model.control["current_servo_monotony"]
        self.active_links_mask = init_world_model.current_world_model.control["active_links_mask"]
        self.min_steps = init_world_model.current_world_model.control["min_steps"]
        self.max_steps = init_world_model.current_world_model.control["max_steps"]
        self.rotating_gripper_servo = init_world_model.current_world_model.control["rotating_gripper_servo"]
        self.gripping_gripper_servo = init_world_model.current_world_model.control["gripping_gripper_servo"]
        self.horizontal_gripper_position = init_world_model.current_world_model.control["horizontal_gripper_position"]
        self.init_position = np.array(init_world_model.current_world_model.control["init_position"]) * self.scale
        self.container_position = \
            np.array(init_world_model.current_world_model.control["container_position"]) * self.scale
        self.init_servo_values = init_world_model.current_world_model.location["init_servo_values"]

        # self.closed_hand_distance_ratio = init_world_model.current_world_model.control["closed_hand_distance_ratio"]
        # self.opened_hand_distance_ratio = init_world_model.current_world_model.control["opened_hand_distance_ratio"]
        # self.url = init_world_model.current_world_model.url["arm"]
        # self.base_put_url = init_world_model.current_world_model.control["base_put_url"]
        # self.send_requests = \
        #     init_world_model.current_world_model.control["send_requests"]  # TODO: make communicator class?
        # self.detect_last_position = init_world_model.current_world_model.control["detect_last_position"]
        # self.verbose = init_world_model.current_world_model.control["verbose"]
        # self.show_plots = init_world_model.current_world_model.control["show_plots"]
        # self.cm_to_servo_polynomial_fitter = \
        #     joblib.load(init_world_model.current_world_model.control["cm_to_servo_polynomial_fitter"]["file_path"])
        # self.scale = init_world_model.current_world_model.control["scale"]
        # self.servo_count = init_world_model.current_world_model.control["servo_count"]
        # self.command_delay = init_world_model.current_world_model.control["command_delay"]  # seconds
        # self.center_init = init_world_model.current_world_model.control["center_init"]
        # self.angle_degree_limit = init_world_model.current_world_model.control["angle_degree_limit"]  # degrees
        # self.trajectory_steps = init_world_model.current_world_model.control["trajectory_steps"]
        # self.current_servo_monotony = init_world_model.current_world_model.control["current_servo_monotony"]
        # self.active_links_mask = init_world_model.current_world_model.control["active_links_mask"]
        # self.min_steps = init_world_model.current_world_model.control["min_steps"]
        # self.max_steps = init_world_model.current_world_model.control["max_steps"]
        # self.rotating_gripper_servo = init_world_model.current_world_model.control["rotating_gripper_servo"]
        # self.gripping_gripper_servo = init_world_model.current_world_model.control["gripping_gripper_servo"]
        # self.horizontal_gripper_position = init_world_model.current_world_model.control["horizontal_gripper_position"]
        # self.init_position = np.array(init_world_model.current_world_model.control["init_position"]) * self.scale
        # self.container_position = \
        #     np.array(init_world_model.current_world_model.control["container_position"]) * self.scale
        # self.init_servo_values = init_world_model.current_world_model.location["init_servo_values"]

        # Link lengths in centimeters
        self.link6 = init_world_model.current_world_model.control["link_lengths"]["link6"]
        self.link5 = init_world_model.current_world_model.control["link_lengths"]["link5"]
        self.link4 = init_world_model.current_world_model.control["link_lengths"]["link4"]
        self.link3 = init_world_model.current_world_model.control["link_lengths"]["link3"]
        self.link2 = init_world_model.current_world_model.control["link_lengths"]["link2"]
        self.link1 = init_world_model.current_world_model.control["link_lengths"]["link1"]

        # Link orientation
        self.orientation6 = init_world_model.current_world_model.control["link_orientations"]["link6"]
        self.orientation5 = init_world_model.current_world_model.control["link_orientations"]["link5"]
        self.orientation4 = init_world_model.current_world_model.control["link_orientations"]["link4"]
        self.orientation3 = init_world_model.current_world_model.control["link_orientations"]["link3"]
        self.orientation2 = init_world_model.current_world_model.control["link_orientations"]["link2"]
        self.orientation1 = init_world_model.current_world_model.control["link_orientations"]["link1"]

        # Joint rotation axis
        self.rotation6 = init_world_model.current_world_model.control["joint_rotation_axis"]["link6"]
        self.rotation5 = init_world_model.current_world_model.control["joint_rotation_axis"]["link5"]
        self.rotation4 = init_world_model.current_world_model.control["joint_rotation_axis"]["link4"]
        self.rotation3 = init_world_model.current_world_model.control["joint_rotation_axis"]["link3"]
        self.rotation2 = init_world_model.current_world_model.control["joint_rotation_axis"]["link2"]
        self.rotation1 = init_world_model.current_world_model.control["joint_rotation_axis"]["link1"]

        # Link bounds (degrees)
        self.bounds6 = np.radians(np.array([-self.angle_degree_limit, self.angle_degree_limit]))
        self.bounds5 = np.radians(np.array([-self.angle_degree_limit, self.angle_degree_limit]))
        self.bounds4 = np.radians(np.array([-self.angle_degree_limit, self.angle_degree_limit]))
        self.bounds3 = np.radians(np.array([-self.angle_degree_limit, self.angle_degree_limit]))
        self.bounds2 = np.radians(np.array([-self.angle_degree_limit, self.angle_degree_limit]))
        self.bounds1 = np.radians(np.array([-self.angle_degree_limit, self.angle_degree_limit]))

        self.chain_name = init_world_model.current_world_model.control["chain_name"]

        self.le_arm_chain = Chain(name=self.chain_name, active_links_mask=self.active_links_mask, links=[
            URDFLink(
                name="link6",
                translation_vector=self.link6 * self.scale,
                orientation=self.orientation6,
                rotation=self.rotation6,
                bounds=self.bounds6
            ),
            URDFLink(
                name="link5",
                translation_vector=self.link5 * self.scale,
                orientation=self.orientation5,
                rotation=self.rotation5,
                bounds=self.bounds5
            ),
            URDFLink(
                name="link4",
                translation_vector=self.link4 * self.scale,
                orientation=self.orientation4,
                rotation=self.rotation4,
                bounds=self.bounds4
            ),
            URDFLink(
                name="link3",
                translation_vector=self.link3 * self.scale,
                orientation=self.orientation3,
                rotation=self.rotation3,
                bounds=self.bounds3
            ),
            URDFLink(
                name="link2",
                translation_vector=self.link2 * self.scale,
                orientation=self.orientation2,
                rotation=self.rotation2,
                bounds=self.bounds2
            ),
            URDFLink(
                name="link1",
                translation_vector=self.link1 * self.scale,
                orientation=self.orientation1,
                rotation=self.rotation1,
                bounds=self.bounds1
            )
        ])

    def xyz_to_servo_range(self, xyz, current_servo_monotony):
        """
        Converts 3D cartesian centimeter coordinates to servo values in [500, 2500].
        :param xyz: Array of 3 elements of a 3D cartesian systems of centimeters.
        :param current_servo_monotony: List of 6 positive or negative servo rotation directions.
        :return: List of 6 servo values in [500, 2500].
        """
        k = self.le_arm_chain.inverse_kinematics(geometry_utils.to_transformation_matrix(xyz, np.eye(3)))
        k = np.multiply(k, np.negative(current_servo_monotony))
        return self.radians_to_servo_range(k)

    def servo_range_to_xyz(self, servo_range, current_servo_monotony):
        """
        Converts servo values in [500, 2500] to  3D cartesian centimeter coordinates.
        :param servo_range: List of 6 servo values in [500, 2500].
        :param current_servo_monotony: List of 6 positive or negative servo rotation directions.
        :return: Array of 3 elements of a 3D cartesian systems of centimeters.
        """
        return geometry_utils.from_transformation_matrix(
            self.le_arm_chain.forward_kinematics(
                np.multiply(self.servo_range_to_radians(servo_range), np.negative(current_servo_monotony)),
            ))[0][:3]

    @staticmethod
    def servo_range_to_radians(x, x_min=500.0, x_max=2500.0, scaled_min=(-np.pi / 2.0), scaled_max=(np.pi / 2.0)):
        """
        Converts servo values in [500, 2500] to angle radians.
        :param x: List of 6 servo values.
        :param x_min: Scalar float, minimum servo value of 90 degrees angle (default = 500).
        :param x_max: Scalar float, maximum servo value of 90 degrees angle(default = 2500).
        :param scaled_min: Scalar float, minimum radians value of +90 degrees angle(default = -π/2).
        :param scaled_max: Scalar float, maximum radians value of +90 degrees angle(default = π/2).
        :return: List of 6 angles in radians.
        """
        x_std = (np.array(x) - x_min) / (x_max - x_min)
        return x_std * (scaled_max - scaled_min) + scaled_min

    @staticmethod
    def radians_to_servo_range(x, x_min=(-np.pi / 2.0), x_max=(np.pi / 2.0), scaled_min=500.0, scaled_max=2500.0):
        """
        Converts angle radians to servo values in [500, 2500].
        :param x: List of 6 angles in radians.
        :param x_min: Scalar float, minimum radians value of +90 degrees angle(default = -π/2).
        :param x_max: Scalar float, maximum radians value of +90 degrees angle(default = π/2).
        :param scaled_min: Scalar float, minimum servo value of 90 degrees angle (default = 500).
        :param scaled_max: Scalar float, maximum servo value of 90 degrees angle(default = 2500).
        :return: List of 6 servo values.
        """
        x_std = (np.array(x) - x_min) / (x_max - x_min)
        return (np.round(x_std * (scaled_max - scaled_min) + scaled_min, 0)).astype(int)

    def get_kinematic_angle_trajectory(self, from_angle_radians_in, to_angle_radians_in, servo_monotony,  steps=10):
        """
        Creates a discrete end-effector trajectory, using radians.
        :param from_angle_radians_in: Current servo angles, list of 6 angles in radians.
        :param to_angle_radians_in: Desired servo angles, list of 6 angles in radians.
        :param servo_monotony: List of 6 positive or negative servo rotation directions.
        :param steps: Scalar integer, the total steps for the end effector trajectory.
        :return: List of end-effector radian trajectory steps.
        """
        assert self.min_steps < steps < self.max_steps

        from_angle_radians = np.multiply(from_angle_radians_in, servo_monotony)
        to_angle_radians = np.multiply(to_angle_radians_in, servo_monotony)

        step_angle_radians = []
        for index in range(len(to_angle_radians_in)):
            step_angle_radians.append((from_angle_radians[index] - to_angle_radians[index]) / float(steps))

        angle_trajectory = []
        step_angle_radians = np.array(step_angle_radians)
        current_angles = np.array(from_angle_radians)
        # angle_trajectory.append(current_angles)

        for _ in range(steps):
            current_angles = np.add(current_angles, step_angle_radians)
            angle_trajectory.append(current_angles)

        return angle_trajectory

    def get_servo_range_trajectory(self, from_servo_range_in, to_servo_range_in, steps=10):
        """
        Creates a discrete end-effector trajectory, using servo values.
        :param from_servo_range_in: Current servo values, list of 6 values in [500, 2500].
        :param to_servo_range_in: Desired servo values, list of 6 values in [500, 2500].
        :param steps: Scalar integer, the total steps for the end effector trajectory.
        :return: List of end-effector servo value trajectory steps.
        """
        assert self.min_steps < steps < self.max_steps

        from_servo_range = np.array(from_servo_range_in)
        to_servo_range = np.array(to_servo_range_in)
        if self.verbose:
            print("from_servo_range: ", from_servo_range)
            print("to_servo_range: ", to_servo_range)

        step_servo_range = []
        for index in range(len(to_servo_range)):
            step_servo_range.append((to_servo_range[index] - from_servo_range[index]) / float(steps))

        if self.verbose:
            print("step_servo_range: ", step_servo_range)

        servo_range_trajectory = []
        step_servo_range = np.array(step_servo_range)
        current_servo_range = np.array(from_servo_range)

        for _ in range(steps):
            current_servo_range = np.add(current_servo_range, step_servo_range)
            servo_range_trajectory.append(current_servo_range)

        return np.array(np.round(servo_range_trajectory, 0)).astype(int)

    def initialize_arm(self, last_servo_values):
        """
        Moves the end-effector to the (0, 0, 0) position of the 3d cartesian.
        :param last_servo_values: List of the current arm servo positions.
        :return: True if succeeded.
        """
        action_successful = False
        target_position = np.array(self.init_position) * self.scale
        action_successful = self.move_arm(target_position, last_servo_values)
        print("=== Arm initialized")
        return action_successful

    def move_arm_above_xyz(self, xyz, last_servo_values, height):
        """
        Moves the end-effector at a specific 3D cartesian centimeter position, plus extra centimeters high.
        :param xyz: Array of 3 elements of a 3D cartesian systems of centimeters.
        :param last_servo_values: List of the current arm servo positions.
        :param height: Scalar positive float. Desired centimeters above xyz, on the z axis.
        :return: True if succeeded.
        """
        action_successful = False
        xyz[2] = height
        target_position = np.array(xyz) * self.scale
        if self.send_requests:
            action_successful = self.move_arm(target_position, last_servo_values)
        print("=== Arm above object")
        return action_successful

    def move_arm_up(self, last_servo_values, height):
        """
        Moves the end-effector at a specific 3D cartesian centimeter position, plus extra centimeters high.
        :param last_servo_values: List of the current arm servo positions.
        :param height: Scalar positive float. Desired centimeters above xyz, on the z axis.
        :return: True if succeeded.
        """
        action_successful = False
        xyz = np.round(self.servo_range_to_xyz(last_servo_values, self.current_servo_monotony), 2)
        print("last_servo_xyz", xyz)

        xyz[2] = height
        target_position = np.array(xyz) * self.scale
        if self.send_requests:
            action_successful = self.move_arm(target_position, last_servo_values)
        print("=== Arm up")
        return action_successful

    def move_arm_to_object(self, xyz, last_servo_values):
        """
        Moves the end-effector to the object's position of the 3d cartesian.
        :param xyz: Array of 3 elements of a 3D cartesian systems of centimeters.
        :param last_servo_values: List of the current arm servo positions.
        :return: True if succeeded.
        """
        action_successful = False
        target_position = np.array(xyz) * self.scale
        if self.send_requests:
            action_successful = self.move_arm(target_position, last_servo_values)
        print("=== Arm to object")
        return action_successful

    def close_hand(self, object_side_length):
        """
        Closes the gripper enough, to grip an object of a specific length in cm.
        :param object_side_length: Scalar float, object width in centimeters.
        :return: True if succeeded.
        """
        action_successful = False
        closed_hand_distance_ratio = self.closed_hand_distance_ratio
        closed_length = object_side_length * closed_hand_distance_ratio
        servo_range = int(self.cm_to_servo_polynomial_fitter(closed_length))
        if self.verbose:
            print("cm: {}, predicted servo value: {}".format(closed_length, servo_range))
        if self.send_requests:
            action_successful = self.send_restful_servo_range(self.gripping_gripper_servo, servo_range)
        print("=== Gripper closed")
        return action_successful

    def open_hand(self, object_side_length):
        """
        Opens the gripper enough, to fit an object of a specific length in cm.
        :param object_side_length: Scalar float, object width in centimeters.
        :return: True if succeeded.
        """
        action_successful = False
        opened_hand_distance_ratio = self.opened_hand_distance_ratio
        opened_length = object_side_length * opened_hand_distance_ratio
        servo_range = int(self.cm_to_servo_polynomial_fitter(opened_length))
        if self.verbose:
            print("cm: {}, predicted servo value: {}".format(opened_length, servo_range))
        if self.send_requests:
            action_successful = self.send_restful_servo_range(self.gripping_gripper_servo, servo_range)
        print("=== Gripper opened")
        return action_successful

    def send_restful_servo_range(self, servo, range):
        """
        Sends a direct servo value in [500, 2500], to a specific servo in [1, 6].
        :param servo: Scalar integer, the servo id in [1, 6].
        :param range: Scalar integer, servo value in [500, 2500].
        :return: True if succeeded.
        """
        action_successful = False
        url = self.base_put_url.format(self.url, servo, range)
        requests.put(url, data="")
        time.sleep(self.command_delay)
        action_successful = True
        return action_successful

    def send_restful_trajectory_requests(self, kinematic_servo_range_trajectory):
        """
        Sends a full servo value trajectory of discrete steps, to the arm.
        :param kinematic_servo_range_trajectory:
        :return: True if succeeded.
        """
        action_successful = False
        servo_mask = self.active_links_mask  # servo mask

        for step in kinematic_servo_range_trajectory:
            for i in range(len(step)):
                if servo_mask[i]:
                    servo_value = step[i]
                    current_servo = self.servo_count - i
                    url = self.base_put_url.format(self.url, current_servo, servo_value)
                    if self.verbose:
                        print(url)
                    try:
                        r = requests.put(url, data="")
                        if r.status_code != 200:
                            break
                    except Exception as e:
                        print("Exception: {}".format(str(e)))
                    time.sleep(self.command_delay)
            if self.verbose:
                print("")

        action_successful = True
        return action_successful

    def move_arm(self, target_position, last_servo_locations, trajectory_steps=-1):
        """
        Gradually moves the end-effector of the robotic arm, from the latest known servo positions, to a desired
        3D centimeter cartesian position.
        :param target_position: List of 3 values, the desired end-effector, 3D centimeter cartesian position.
        :param last_servo_locations: List of the latest 6 servo values in [500, 2500].
        :param trajectory_steps: Scalar integer, the total steps for the end effector trajectory.
        :return: True if successfully move the arm.
        """
        action_successful = False
        last_servo_values = self.init_position
        if trajectory_steps == -1:
            trajectory_steps = self.trajectory_steps

        # TODO: move last position to world model
        if self.detect_last_position:  # TODO: get last servo values anyway, world model may be old...
            last_servo_values = last_servo_locations
            try:
                if self.send_requests:
                    url = "http://{}/".format(self.url)  # TODO: world  model
                    r = requests.get(url, data="")
                    if r.status_code == 200:
                        result = r.json()["variables"]
                        last_servo_values = np.array(
                            [result["servo6"], result["servo5"], result["servo4"], result["servo3"],
                             result["servo2"], result["servo1"]])

                        if self.verbose:
                            print("last_servo_values: ", last_servo_values)
                            print("last_servo_xyz",
                                  np.round(self.servo_range_to_xyz(last_servo_values, self.current_servo_monotony), 2))
            except Exception as e_pos:
                print("Exception: {}".format(str(e_pos)))

        if self.center_init:
            if self.verbose:
                print("Top position (radians): ",
                      self.le_arm_chain.inverse_kinematics(geometry_utils.to_transformation_matrix(
                          self.init_position,
                          np.eye(3))))

            if self.show_plots:
                ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')
                self.le_arm_chain.plot(self.le_arm_chain.inverse_kinematics(geometry_utils.to_transformation_matrix(
                    self.init_position,
                    np.eye(3))), ax, target=self.init_position)
                matplotlib.pyplot.show()

        init_position2 = self.init_position
        init_angle_radians2 = self.le_arm_chain.inverse_kinematics(geometry_utils.to_transformation_matrix(
            init_position2,
            np.eye(3)))
        from_servo_range = self.radians_to_servo_range(init_angle_radians2)
        if self.detect_last_position:
            from_servo_range = last_servo_values
        to_servo_range = self.xyz_to_servo_range(target_position, self.current_servo_monotony)
        kinematic_servo_range_trajectory = self.get_servo_range_trajectory(from_servo_range, to_servo_range,
                                                                           trajectory_steps)
        if self.verbose:
            print("init_angle_radians2: {}, from_servo_range: {}, to_servo_range: {}, servo_range_trajectory: {}"
                  .format(init_angle_radians2, from_servo_range, to_servo_range, kinematic_servo_range_trajectory))

        if self.show_plots:
            ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')
            self.le_arm_chain.plot(self.le_arm_chain.inverse_kinematics(geometry_utils.to_transformation_matrix(
                target_position,
                np.eye(3))), ax,
                target=target_position)
            matplotlib.pyplot.show()

        if self.send_requests:
            action_successful = self.send_restful_trajectory_requests(kinematic_servo_range_trajectory)

        return action_successful


if __name__ == '__main__':

    # Sequence for testing
    from world_model import WorldModel
    current_world_model = WorldModel()
    control = Control(current_world_model)
    control.send_requests = False
    control.center_init = False
    control.detect_last_position = False
    last_servo_values_testing = current_world_model.current_world_model.location["servo_values"]
    control.initialize_arm(last_servo_values_testing)
    control.open_hand(4.4)
    container_xyz = [-0.1, 25.0, 12]
    control.move_arm(container_xyz, last_servo_values_testing)
    control.close_hand(4.4)
    # target_position = np.array([12.5, -12.5, 2.0]) * coordination.control.scale
    target_position_testing = np.array([20, -20.0, 20]) * control.scale
    # target_position = np.array([12.5, -12.5, 25]) * coordination.control.scale
    # target_position = np.array([-16, 0.0, 10]) * coordination.control.scale
    # target_position = np.array([-20, -20, 25]) * coordination.control.scale
    # target_position = np.array([0, 0, 0]) * coordination.control.scale
    # target_position = np.array([-13.12, 0.27, 1.5]) * coordination.control.scale
    action_successful_testing = control.move_arm(np.array(target_position_testing), last_servo_values_testing)
