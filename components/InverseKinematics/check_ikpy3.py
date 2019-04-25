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
bounds1 = np.radians(np.array([-60, 60]))
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

show_init = False

if show_init:
    target_position = [0, 0, 1.0]
    print("Top position: ", left_arm_chain.inverse_kinematics(geometry_utils.to_transformation_matrix(
        target_position,
        np.eye(3))))
    ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')

    left_arm_chain.plot(left_arm_chain.inverse_kinematics(geometry_utils.to_transformation_matrix(
        target_position,
        np.eye(3))), ax, target=target_position)
    matplotlib.pyplot.show()


# target_position = [0.5, 0.5, 0.0]
# target_position = [.8, .8, .8]
target_position = [.5, -.75, 1]
print("Target position: ", left_arm_chain.inverse_kinematics(geometry_utils.to_transformation_matrix(
    target_position,
    np.eye(3))))
ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')

left_arm_chain.plot(left_arm_chain.inverse_kinematics(geometry_utils.to_transformation_matrix(
    target_position,
    np.eye(3))), ax,
    target=target_position)
matplotlib.pyplot.show()

