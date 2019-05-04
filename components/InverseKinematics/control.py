from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import matplotlib.pyplot
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from ikpy import geometry_utils
import requests
import time


class Control:

    def __init__(self):
        pass

    def xyz_to_servo_range(self, xyz, current_servo_monotony):
        k = le_arm_chain.inverse_kinematics(geometry_utils.to_transformation_matrix(xyz, np.eye(3)))
        k = np.multiply(k, np.negative(current_servo_monotony))
        return self.radians_to_servo_range(k)

    def servo_range_to_xyz(self, servo_range, current_servo_monotony):
        return geometry_utils.from_transformation_matrix(
            le_arm_chain.forward_kinematics(
            np.multiply(self.servo_range_to_radians(servo_range), np.negative(current_servo_monotony)),
            ))[0][:3]

    def xyz_to_servo_range2(self, xyz, current_servo_monotony):
        k = le_arm_chain.inverse_kinematics(geometry_utils.to_transformation_matrix(xyz, np.eye(3)))
        k = np.multiply(k, current_servo_monotony)
        return self.radians_to_servo_range(k)

    def servo_range_to_xyz2(self, servo_range, current_servo_monotony):
        return geometry_utils.from_transformation_matrix(
            le_arm_chain.forward_kinematics(
            np.multiply(self.servo_range_to_radians(servo_range), current_servo_monotony),
            ))[0][:3]

    def servo_range_to_radians(self, x, x_min=500.0, x_max=2500.0, scaled_min=(-np.pi / 2.0), scaled_max=(np.pi / 2.0)):
        x_std = (np.array(x) - x_min) / (x_max - x_min)
        return x_std * (scaled_max - scaled_min) + scaled_min

    def radians_to_servo_range(self, x, x_min=(-np.pi / 2.0), x_max=(np.pi / 2.0), scaled_min=500.0, scaled_max=2500.0):
        x_std = (np.array(x) - x_min) / (x_max - x_min)
        return (np.round(x_std * (scaled_max - scaled_min) + scaled_min, 0)).astype(int)

    def get_kinematic_angle_trajectory(self, from_angle_radians_in, to_angle_radians_in, servo_monotony, steps=10):
        assert min_steps < steps < max_steps

        from_angle_radians = np.multiply(from_angle_radians_in, servo_monotony)
        to_angle_radians = np.multiply(to_angle_radians_in, servo_monotony)

        step_angle_radians = []
        for index in range(len(target_angle_radians)):
            step_angle_radians.append((from_angle_radians[index] - to_angle_radians[index]) / float(steps))

        angle_trajectory = []
        step_angle_radians = np.array(step_angle_radians)
        current_angles = np.array(from_angle_radians)
        # angle_trajectory.append(current_angles)

        for _ in range(steps):
            current_angles = np.add(current_angles, step_angle_radians)
            angle_trajectory.append(current_angles)

        return angle_trajectory


    def get_kinematic_servo_trajectory(self, from_servo_values, to_servo_values, steps=10):
        assert min_steps < steps < max_steps

        print("from_servo_values: ", from_servo_values)
        print("to_servo_values: ", to_servo_values)

        step_servo_values = (from_servo_values - to_servo_values) / float(steps)

        servo_values_trajectory = []
        step_servo_values = np.array(step_servo_values)
        current_angles = np.array(from_servo_values)

        for _ in range(steps):
            current_angles = np.add(current_angles, step_servo_values)
            servo_values_trajectory.append(current_angles)

        return (np.round(servo_values_trajectory, 0)).astype(int)


if __name__ == '__main__':

    control = Control()

    send_requests = True
    # send_requests = False
    # scale = 0.04  # For the plotting
    scale = 1.0
    servo_count = 6
    command_delay = 0.05  # seconds
    center_init = True
    # center_init = False
    angle_degree_limit = 75  # degrees
    trajectory_steps = 10
    current_servo_monotony = [-1.0, -1.0, 1.0, -1.0, -1.0, -1.0]
    active_links_mask = [True, True, True, True, False, True]  # Enabled/disabled links
    min_steps = 1
    max_steps = 5000
    gripper_servo = 2
    gripper_open = 600

    # target_position = np.array([12.5, -12.5, 2.0]) * scale
    # target_position = np.array([20, -20.0, 20]) * scale
    # target_position = np.array([12.5, -12.5, 25]) * scale
    # target_position = np.array([-5, -5, 40]) * scale
    # target_position = np.array([-16, 0.0, 10]) * scale
    # target_position = np.array([-20, -20, 25]) * scale
    target_position = np.array([0, 0, 0]) * scale
    # target_position = np.array([-13.12, 0.27, 1.5]) * scale

    init_position = np.array([0, 0, 1]) * scale

    # Link lengths in centimeters
    link6 = np.array([0, 0, 7.0])
    link5 = np.array([0, 0, 3.0])
    link4 = np.array([0, 0, 10.5])
    link3 = np.array([0, 0, 9.0])
    link2 = np.array([0, 0, 7.0])
    link1 = np.array([0, 0, 10.0])

    # Joint rotation axis
    rotation6 = np.array([0, 0, 1])
    rotation5 = np.array([0, 1, 0])
    rotation4 = np.array([0, 1, 0])
    rotation3 = np.array([0, 1, 0])
    rotation2 = np.array([0, 0, 1])
    rotation1 = np.array([0, 0, 1])

    # Link bounds (degrees)  # TODO: per servo bounds
    bounds6 = np.radians(np.array([-angle_degree_limit, angle_degree_limit]))
    bounds5 = np.radians(np.array([-angle_degree_limit, angle_degree_limit]))
    bounds4 = np.radians(np.array([-angle_degree_limit, angle_degree_limit]))
    bounds3 = np.radians(np.array([-angle_degree_limit, angle_degree_limit]))
    bounds2 = np.radians(np.array([-angle_degree_limit, angle_degree_limit]))
    bounds1 = np.radians(np.array([-angle_degree_limit, angle_degree_limit]))

    le_arm_chain = Chain(name='le_arm', active_links_mask=active_links_mask, links=[
        URDFLink(
            name="link6",
            translation_vector=link6 * scale,
            orientation=[0, 0, 0],
            rotation=rotation6,
            bounds=bounds6
        ),
        URDFLink(
            name="link5",
            translation_vector=link5 * scale,
            orientation=[0, 0, 0],
            rotation=rotation5,
            bounds=bounds5
        ),
        URDFLink(
            name="link4",
            translation_vector=link4 * scale,
            orientation=[0, 0, 0],
            rotation=rotation4,
            bounds=bounds4
        ),
        URDFLink(
            name="link3",
            translation_vector=link3 * scale,
            orientation=[0, 0, 0],
            rotation=rotation3,
            bounds=bounds3
        ),
        URDFLink(
            name="link2",
            translation_vector=link2 * scale,
            orientation=[0, 0, 0],
            rotation=rotation2,
            bounds=bounds2
        ),
        URDFLink(
            name="link1",
            translation_vector=link1 * scale,
            orientation=[0, 0, 0],
            rotation=rotation1,
            bounds=bounds1
        )
    ])

    # TODO: init from request
    detect_last_position = False
    init_servo_values = [1500, 1500, 1500, 1500, 1500, 1500]  # TODO: temp
    if detect_last_position:
        try:
            if send_requests:
                url = "http://ESP32/"
                r = requests.put(url, data="")
                # print("r.status_code: ", r.status_code)
                # print("r.text: ", r.text)
                # print("r.encoding: ", r.encoding)
                # print("r.json(): ", r.json())
                # print("r.headers['content-type']: ", r.headers['content-type'])
                # print("servo6: ", r.json()['variables']['servo6'])
                result = r.json()['variables']
                # print("servo6: ", result['servo6'], result['servo5'])
                if r.status_code == 200:
                    result = r.json()["variables"]
                    init_servo_values = np.array([result["servo6"], result["servo5"], result["servo4"], result["servo3"],
                                                  result["servo2"], result["servo1"]])

                    # init_servo_radians = np.multiply(servo_range_to_radians(init_servo_values), current_servo_monotony)
                    # print("init_servo_radians: ", init_servo_radians)
                    #
                    # print("init_servo_radians: ", np.multiply(servo_range_to_radians(init_servo_values),
                    #                                          current_servo_monotony[::-1]))
                    #
                    # init_position2 = le_arm_chain.forward_kinematics(init_servo_radians)
                    # init_position = np.round(servo_range_to_xyz2(init_servo_values, current_servo_monotony), 2)
                    # print("predicted_init_position: ", init_position)
                    # print("proper init_position: ", target_position)

        except Exception as e:
            print("Exception: {}".format(str(e)))

    if center_init:
        print("Top position (radians): ", le_arm_chain.inverse_kinematics(geometry_utils.to_transformation_matrix(
            init_position,
            np.eye(3))))
        ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')

        le_arm_chain.plot(le_arm_chain.inverse_kinematics(geometry_utils.to_transformation_matrix(
            init_position,
            np.eye(3))), ax, target=init_position)
        matplotlib.pyplot.show()

    print("Target angles (radians): ", le_arm_chain.inverse_kinematics(geometry_utils.to_transformation_matrix(
        target_position,
        np.eye(3))))
    ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')

    le_arm_chain.plot(le_arm_chain.inverse_kinematics(geometry_utils.to_transformation_matrix(
        target_position,
        np.eye(3))), ax,
        target=target_position)
    matplotlib.pyplot.show()


    target_angle_radians = le_arm_chain.inverse_kinematics(geometry_utils.to_transformation_matrix(
            target_position,
            np.eye(3)))

    # TODO: test 0
    # target_servo_range = radians_to_servo_range(np.multiply(target_angle_radians, current_servo_monotony))
    # kinematic_servo_range_trajectory = get_kinematic_servo_trajectory(init_servo_values, target_servo_range, trajectory_steps)
    # print("kinematic_servo_range_trajectory (steps: {}): {}".format(trajectory_steps, kinematic_servo_range_trajectory))

    # TODO: test 1
    init_angle_radians = le_arm_chain.inverse_kinematics(geometry_utils.to_transformation_matrix(
            init_position,
            np.eye(3)))
    kinematic_angle_trajectory = control.get_kinematic_angle_trajectory(init_angle_radians, target_angle_radians,
                                                                current_servo_monotony, trajectory_steps)
    print("kinematic_angle_trajectory (steps: {}): {}".format(trajectory_steps, kinematic_angle_trajectory))
    print("kinematic_angle_trajectory (steps: {}): {}".format(trajectory_steps, np.rad2deg(kinematic_angle_trajectory)))
    kinematic_servo_range_trajectory = control.radians_to_servo_range(kinematic_angle_trajectory)
    print("kinematic_servo_range_trajectory (steps: {}): {}".format(trajectory_steps, kinematic_servo_range_trajectory))


    # TODO: from to, to-from with MONOTONY
    servo_range1 = control.xyz_to_servo_range(target_position, current_servo_monotony)
    target2 = np.round(control.servo_range_to_xyz(servo_range1, current_servo_monotony), 2)
    print("\n", target_position, " -> \n",
          servo_range1, " -> \n",
          target2, "\n")

    # TODO: https

    # TODO: AR ocv virtual grid on camera
    servo_mask = active_links_mask  # TODO: servo mask

    if send_requests:

        url = "http://ESP32/set_servo{}?value={}".format(gripper_servo, gripper_open)  # TODO: gripper horizontal orientation
        requests.put(url, data="")
        time.sleep(command_delay)

        for step in kinematic_servo_range_trajectory:
            for i in range(len(step)):
                if servo_mask[i]:
                    servo_value = step[i]
                    current_servo = servo_count - i
                    if current_servo == 1 and servo_value < 1500:  # Gripper MUST be >= 1500
                        servo_value = 1500
                    url = "http://ESP32/set_servo{}?value={}".format(current_servo, servo_value)
                    print(url)
                    try:
                        r = requests.put(url, data="")
                        if r.status_code != 200:
                            break  # TODO: abort
                    except Exception as e:
                        print("Exception: {}".format(str(e)))
                    time.sleep(command_delay)
            print("")
