from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import matplotlib.pyplot
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from ikpy import geometry_utils


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

# target_position = [0.5, 0.5, 0.0]
# target_position = [.8, .8, .8]
# target_position = [.5, .5, 1]
target_position = [.8, .8, -.9]
print("Target angles (radians): ", left_arm_chain.inverse_kinematics(geometry_utils.to_transformation_matrix(
    target_position,
    np.eye(3))))
ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')

left_arm_chain.plot(left_arm_chain.inverse_kinematics(geometry_utils.to_transformation_matrix(
    target_position,
    np.eye(3))), ax,
    target=target_position)
matplotlib.pyplot.show()

#
# def get_kinematic_angle_trajectory(from_angle_randians, to_angle_radians, steps=10):
#     assert 1 < steps < 5000
#     x, y, z = from_angle_randians
#     x2, y2, z2 = to_angle_radians
#
#     angle_trajectory = []
#     step_size_x = (x2 - x) / float(steps)
#     step_size_y = (y2 - y) / float(steps)
#     step_size_z = (z2 - z) / float(steps)
#
#     x_new, y_new, z_new = x, y, z
#     angle_trajectory.append((x_new, y_new, z_new))
#     for step in steps - 1:
#         x_new += step_size_x
#         y_new += step_size_y
#         z_new += step_size_z
#         angle_trajectory.append(x_new, y_new, z_new)
#
#     return angle_trajectory


def get_kinematic_angle_trajectory(from_angle_radians, to_angle_radians, steps=10):
    assert 1 < steps < 5000

    step_angle_radians = []
    for i in range(len(target_angle_radians)):
        step_angle_radians.append(
            (from_angle_radians[i] - to_angle_radians[i]) / float(steps)
        )

    step_angle_radians = np.array(step_angle_radians)

    angle_trajectory = []
    current_angles = np.array(from_angle_radians)

    for _ in range(steps):
        current_angles = np.add(current_angles, step_angle_radians)
        angle_trajectory.append(current_angles)

    return angle_trajectory


steps = 5
init_angle_radians = left_arm_chain.inverse_kinematics(geometry_utils.to_transformation_matrix(
        init_position,
        np.eye(3)))
target_angle_radians = left_arm_chain.inverse_kinematics(geometry_utils.to_transformation_matrix(
        target_position,
        np.eye(3)))

print("Kinematic trajectory (steps: {}): {}".format(steps, get_kinematic_angle_trajectory(init_angle_radians,
                                                                                          target_angle_radians, steps)))
