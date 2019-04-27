from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import matplotlib.pyplot
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from ikpy import geometry_utils
import requests
import time


# Link lengths in centimeters
link1 = np.array([0, 0, 8.0])
link2 = np.array([0, 0, 1.0])
link3 = np.array([0, 0, 12.0])
link4 = np.array([0, 0, 9.0])
link5 = np.array([0, 0, 6.0])
link6 = np.array([0, 0, 6.0])

scale = 0.04

# Joint rotation axis
rotation1 = np.array([0, 0, 1])
rotation2 = np.array([0, 1, 0])
rotation3 = np.array([0, 1, 0])
rotation4 = np.array([0, 1, 0])
rotation5 = np.array([0, 1, 0])
rotation6 = np.array([0, 0, 1])

# Link bounds (degrees)
bounds1 = np.radians(np.array([-60, 60]))  # TODO: increase the z angles?
bounds2 = np.radians(np.array([-60, 60]))
bounds3 = np.radians(np.array([-60, 60]))
bounds4 = np.radians(np.array([-60, 60]))
bounds5 = np.radians(np.array([-60, 60]))
bounds6 = np.radians(np.array([-60, 60]))

# Enabled/disabled links
active_links_mask = [True, True, True, True, True, True]

left_arm_chain = Chain(name='left_arm', active_links_mask=active_links_mask, links=[
    URDFLink(
      name="shoulder",
      translation_vector=link1 * scale,
      orientation=[0, 0, 0],
      rotation=rotation1,
      bounds=bounds1
    ),
    URDFLink(
      name="elbow",
      translation_vector=link2 * scale,
      orientation=[0, 0, 0],
      rotation=rotation2,
      bounds=bounds2
    ),
    URDFLink(
      name="wrist",
      translation_vector=link3 * scale,
      orientation=[0, 0, 0],
      rotation=rotation3,
      bounds=bounds3
    ),
    URDFLink(
      name="wrist",
      translation_vector=link4 * scale,
      orientation=[0, 0, 0],
      rotation=rotation4,
      bounds=bounds4
    ),
    URDFLink(
      name="wrist",
      translation_vector=link5 * scale,
      orientation=[0, 0, 0],
      rotation=rotation5,
      bounds=bounds5
    ),
    URDFLink(
      name="wrist2",
      translation_vector=link6 * scale,
      orientation=[0, 0, 0],
      rotation=rotation6,
      bounds=bounds6
    )
])

show_init = True
send_requests = False

if show_init:
    init_position = [0, 0, 1]
    target_position = init_position
    # target_position = [.08, .08, 4]
    print("Top position (radians): ", left_arm_chain.inverse_kinematics(geometry_utils.to_transformation_matrix(
        target_position,
        np.eye(3))))
    ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')

    left_arm_chain.plot(left_arm_chain.inverse_kinematics(geometry_utils.to_transformation_matrix(
        target_position,
        np.eye(3))), ax, target=target_position)
    matplotlib.pyplot.show()

target_position = [0.5, 0.5, 0.0]
# target_position = [.8, .8, .8]
# target_position = [.5, .5, 1]
# target_position = [.8, .8, 1]
print("Target angles (radians): ", left_arm_chain.inverse_kinematics(geometry_utils.to_transformation_matrix(
    target_position,
    np.eye(3))))
ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')

left_arm_chain.plot(left_arm_chain.inverse_kinematics(geometry_utils.to_transformation_matrix(
    target_position,
    np.eye(3))), ax,
    target=target_position)
matplotlib.pyplot.show()


def radians_to_servo_range(x, x_min=-np.pi, x_max=np.pi, scaled_min=500.0, scaled_max=2500.0):
    x = np.array(x)
    x_std = (x - x_min) / (x_max - x_min)
    return (x_std * (scaled_max - scaled_min) + scaled_min).astype(int)


def get_kinematic_angle_trajectory(from_angle_radians, to_angle_radians, servo_monotony, steps=10):
    assert 1 < steps < 5000

    step_angle_radians = []
    for index in range(len(target_angle_radians)):
        step_angle_radians.append((from_angle_radians[index] - to_angle_radians[index]) / float(steps))

    angle_trajectory = []
    step_angle_radians = np.array(step_angle_radians)
    current_angles = np.array(from_angle_radians)
    current_angles = np.multiply(current_angles, servo_monotony)
    # angle_trajectory.append(current_angles)

    for _ in range(steps):
        current_angles = np.add(current_angles, step_angle_radians)
        current_angles = np.multiply(current_angles, servo_monotony)
        angle_trajectory.append(current_angles)

    return angle_trajectory


trajectory_steps = 5
init_angle_radians = left_arm_chain.inverse_kinematics(geometry_utils.to_transformation_matrix(
        init_position,
        np.eye(3)))
target_angle_radians = left_arm_chain.inverse_kinematics(geometry_utils.to_transformation_matrix(
        target_position,
        np.eye(3)))

# TODO: servo mask
servo_mask = [True, True, True, True, False, False]
current_servo_monotony = [1.0, 1.0, -1.0, 1.0, 1.0, 1.0]

kinematic_angle_trajectory = get_kinematic_angle_trajectory(init_angle_radians, target_angle_radians,
                                                            current_servo_monotony, trajectory_steps)
print("kinematic_angle_trajectory (steps: {}): {}".format(trajectory_steps, kinematic_angle_trajectory))

print("kinematic_angle_trajectory (steps: {}): {}".format(trajectory_steps, np.rad2deg(kinematic_angle_trajectory)))

kinematic_servo_range_trajectory = radians_to_servo_range(kinematic_angle_trajectory)
print("kinematic_servo_range_trajectory (steps: {}): {}".format(trajectory_steps, kinematic_servo_range_trajectory))

servo_count = 6

if send_requests:
    for step in kinematic_servo_range_trajectory:
        for i in range(len(step)):
            if servo_mask[i]:
                servo_value = step[i]
                if servo_value < 500 and servo_value > 2500:
                    servo_value = 1550  # TODO: change
                current_servo = servo_count - i
                url = "http://ESP_02662E/set_servo{}?value={}".format(current_servo, servo_value)
                print(url)
                try:
                    # response = requests.put(url, data="")
                    if send_requests:
                        requests.put(url, data="")
                except Exception as e:
                    print("Exception: {}".format(str(e)))
                time.sleep(0.05)
                time.sleep(2)
        print("")
