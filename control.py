from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import matplotlib.pyplot
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from ikpy import geometry_utils
import requests
import time
from sklearn.externals import joblib


class Control:

    def __init__(self):

        # send_requests = True
        self.send_requests = False
        self.detect_last_position = False
        self.verbose = False
        self.show_plots = False
        self.cm_to_servo_polynomial_fitter = joblib.load('modelsQr/cm_to_servo_polynomial_fitter.sav')
        # self.scale = 0.04  # For the plotting
        self.scale = 1.0
        self.servo_count = 6
        self.command_delay = 0.1  # seconds
        self.center_init = True
        self.closed_hand_distance_ratio = 0.8
        self.opened_hand_distance_ratio = 1.2
        # self.center_init = False
        self.angle_degree_limit = 75  # degrees
        self.trajectory_steps = 10
        self.current_servo_monotony = [-1.0, -1.0, 1.0, -1.0, -1.0, -1.0]
        self.active_links_mask = [True, True, True, True, False, True]  # Enabled/disabled links
        self.min_steps = 1
        self.max_steps = 5000
        self.rotating_gripper_servo = 2
        self.gripping_gripper_servo = 1
        self.horizontal_gripper_position = 600
        self.init_position = np.array([0, 0, 1]) * self.scale
        self.container_position = np.array([0, 18, 10]) * self.scale
        self.init_servo_values = [1500, 1500, 1500, 1500, 1500, 1500]  # TODO: temp

        # Link lengths in centimeters
        self.link6 = np.array([0, 0, 7.0])
        self.link5 = np.array([0, 0, 3.0])
        self.link4 = np.array([0, 0, 10.5])
        self.link3 = np.array([0, 0, 9.0])
        self.link2 = np.array([0, 0, 7.0])
        self.link1 = np.array([0, 0, 10.0])

        # Joint rotation axis
        self.rotation6 = np.array([0, 0, 1])
        self.rotation5 = np.array([0, 1, 0])
        self.rotation4 = np.array([0, 1, 0])
        self.rotation3 = np.array([0, 1, 0])
        self.rotation2 = np.array([0, 0, 1])
        self.rotation1 = np.array([0, 0, 1])

        # Link bounds (degrees)  # TODO: per servo bounds
        self.bounds6 = np.radians(np.array([-self.angle_degree_limit, self.angle_degree_limit]))
        self.bounds5 = np.radians(np.array([-self.angle_degree_limit, self.angle_degree_limit]))
        self.bounds4 = np.radians(np.array([-self.angle_degree_limit, self.angle_degree_limit]))
        self.bounds3 = np.radians(np.array([-self.angle_degree_limit, self.angle_degree_limit]))
        self.bounds2 = np.radians(np.array([-self.angle_degree_limit, self.angle_degree_limit]))
        self.bounds1 = np.radians(np.array([-self.angle_degree_limit, self.angle_degree_limit]))

        self.le_arm_chain = Chain(name='le_arm', active_links_mask=self.active_links_mask, links=[
            URDFLink(
                name="link6",
                translation_vector=self.link6 * self.scale,
                orientation=[0, 0, 0],
                rotation=self.rotation6,
                bounds=self.bounds6
            ),
            URDFLink(
                name="link5",
                translation_vector=self.link5 * self.scale,
                orientation=[0, 0, 0],
                rotation=self.rotation5,
                bounds=self.bounds5
            ),
            URDFLink(
                name="link4",
                translation_vector=self.link4 * self.scale,
                orientation=[0, 0, 0],
                rotation=self.rotation4,
                bounds=self.bounds4
            ),
            URDFLink(
                name="link3",
                translation_vector=self.link3 * self.scale,
                orientation=[0, 0, 0],
                rotation=self.rotation3,
                bounds=self.bounds3
            ),
            URDFLink(
                name="link2",
                translation_vector=self.link2 * self.scale,
                orientation=[0, 0, 0],
                rotation=self.rotation2,
                bounds=self.bounds2
            ),
            URDFLink(
                name="link1",
                translation_vector=self.link1 * self.scale,
                orientation=[0, 0, 0],
                rotation=self.rotation1,
                bounds=self.bounds1
            )
        ])

    def xyz_to_servo_range(self, xyz, current_servo_monotony):
        k = self.le_arm_chain.inverse_kinematics(geometry_utils.to_transformation_matrix(xyz, np.eye(3)))
        k = np.multiply(k, np.negative(current_servo_monotony))
        return self.radians_to_servo_range(k)

    def servo_range_to_xyz(self, servo_range, current_servo_monotony):
        return geometry_utils.from_transformation_matrix(
            self.le_arm_chain.forward_kinematics(
                np.multiply(self.servo_range_to_radians(servo_range), np.negative(current_servo_monotony)),
            ))[0][:3]

    @staticmethod
    def servo_range_to_radians(x, x_min=500.0, x_max=2500.0, scaled_min=(-np.pi / 2.0), scaled_max=(np.pi / 2.0)):
        x_std = (np.array(x) - x_min) / (x_max - x_min)
        return x_std * (scaled_max - scaled_min) + scaled_min

    @staticmethod
    def radians_to_servo_range(x, x_min=(-np.pi / 2.0), x_max=(np.pi / 2.0), scaled_min=500.0, scaled_max=2500.0):
        x_std = (np.array(x) - x_min) / (x_max - x_min)
        return (np.round(x_std * (scaled_max - scaled_min) + scaled_min, 0)).astype(int)

    def get_kinematic_angle_trajectory(self, from_angle_radians_in, to_angle_radians_in, servo_monotony,  steps=10):
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

    def initialize_arm(self):
        action_successful = False
        target_position = np.array(self.init_position) * self.scale
        action_successful = self.move_arm(target_position)
        print("=== Arm initialized")

        return action_successful

    def move_arm_to_container(self, xyz):
        action_successful = False
        # target_position = np.array([-0.1, 22.0, 10]) * self.scale
        target_position = np.array(xyz) * self.scale
        # np.array([0, 18, 10]) * self.scale
        action_successful = self.move_arm(target_position)
        print("=== Arm to container")

        return action_successful

    def move_arm_to_object(self, xyz):
        action_successful = False
        # target_position = np.array([-12.5, -12.5, 5]) * self.scale
        target_position = np.array(xyz) * self.scale
        # np.array([0, 18, 10]) * self.scale
        action_successful = self.move_arm(target_position)
        print("=== Arm to container")

        return action_successful

    def close_hand(self):
        action_successful = False
        object_side_length = 4.4
        closed_hand_distance_ratio = 0.8

        closed_length = object_side_length * closed_hand_distance_ratio
        servo_range = int(self.cm_to_servo_polynomial_fitter(closed_length))
        if self.verbose:
            print("cm: {}, predicted servo value: {}".format(closed_length, servo_range))

        action_successful = self.send_restful_servo_range(self.gripping_gripper_servo, servo_range)

        print("=== Gripper closed")

        return action_successful

    def open_hand(self):
        action_successful = False
        object_side_length = 4.4
        opened_hand_distance_ratio = 1.2

        opened_length = object_side_length * opened_hand_distance_ratio
        servo_range = int(self.cm_to_servo_polynomial_fitter(opened_length))
        if self.verbose:
            print("cm: {}, predicted servo value: {}".format(opened_length, servo_range))

        action_successful = self.send_restful_servo_range(self.gripping_gripper_servo, servo_range)

        print("=== Gripper opened")

        return action_successful

    def send_restful_servo_range(self, servo, range):
        action_successful = False
        url = "http://ESP32/set_servo{}?value={}".format(servo, range)  # TODO: gripper horizontal orientation
        requests.put(url, data="")
        time.sleep(self.command_delay)
        action_successful = True
        return action_successful

    def send_restful_trajectory_requests(self, kinematic_servo_range_trajectory):
        action_successful = False
        servo_mask = self.active_links_mask  # servo mask

        for step in kinematic_servo_range_trajectory:
            for i in range(len(step)):
                if servo_mask[i]:
                    servo_value = step[i]
                    current_servo = self.servo_count - i
                    if current_servo == 1 and servo_value < 1500:  # Gripper MUST be >= 1500
                        servo_value = 1500
                    url = "http://ESP32/set_servo{}?value={}".format(current_servo, servo_value)
                    if self.verbose:
                        print(url)
                    try:
                        r = requests.put(url, data="")
                        if r.status_code != 200:
                            break  # TODO: abort
                    except Exception as e:
                        print("Exception: {}".format(str(e)))
                    time.sleep(self.command_delay)
            if self.verbose:
                print("")

        action_successful = True
        return action_successful

    def move_arm(self, target_position):
        action_successful = False

        # TODO: init from request
        if self.detect_last_position:
            last_servo_values = self.init_position
            try:
                if self.send_requests:
                    url = "http://ESP32/"
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

        # TODO: from to servo range -> trajectory
        init_position2 = self.init_position
        # init_position2 = last_servo_values
        init_angle_radians2 = self.le_arm_chain.inverse_kinematics(geometry_utils.to_transformation_matrix(
            init_position2,
            np.eye(3)))
        from_servo_range = self.radians_to_servo_range(init_angle_radians2)
        if self.detect_last_position:
            from_servo_range = last_servo_values
        to_servo_range = self.xyz_to_servo_range(target_position, self.current_servo_monotony)
        kinematic_servo_range_trajectory = self.get_servo_range_trajectory(from_servo_range, to_servo_range,
                                                                           self.trajectory_steps)
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
    control = Control()
    # control.send_requests = False
    control.center_init = False
    # control.detect_last_position = False
    control.initialize_arm()
    control.open_hand()
    control.move_arm_to_container()
    control.close_hand()

    # target_position = np.array([12.5, -12.5, 2.0]) * monitoring.control.scale
    target_position = np.array([20, -20.0, 20]) * control.scale
    # target_position = np.array([12.5, -12.5, 25]) * monitoring.control.scale
    # target_position = np.array([-16, 0.0, 10]) * monitoring.control.scale
    # target_position = np.array([-20, -20, 25]) * monitoring.control.scale
    # target_position = np.array([0, 0, 0]) * monitoring.control.scale
    # target_position = np.array([-13.12, 0.27, 1.5]) * monitoring.control.scale
    action_successful = control.move_arm(np.array(target_position))
