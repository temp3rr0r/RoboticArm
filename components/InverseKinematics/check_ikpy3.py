from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import matplotlib.pyplot
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from ikpy import geometry_utils


# Link lengths in centimeters
link1 = np.array([0, 0, 10.0])
link2 = np.array([0, 0, 12.0])
link3 = np.array([0, 0, 9.0])
link4 = np.array([0, 0, 6.0])
link5 = np.array([0, 0, 12.0])

scale = 0.04

# Joint rotation axis
rotation1 = np.array([0, 0, 1])
rotation2 = np.array([0, 1, 0])
rotation3 = np.array([0, 1, 0])
rotation4 = np.array([0, 1, 0])
rotation5 = np.array([0, 0, 1])

left_arm_chain = Chain(name='left_arm', links=[
    OriginLink(),
    URDFLink(
      name="shoulder",
      translation_vector=link1 * scale,
      orientation=[0, 0, 0],
      rotation=rotation1
    ),
    URDFLink(
      name="elbow",
      translation_vector=link2 * scale,
      orientation=[0, 0, 0],
      rotation=rotation2,
    ),
    URDFLink(
      name="wrist",
      translation_vector=link3 * scale,
      orientation=[0, 0, 0],
      rotation=rotation3
    ),
    URDFLink(
      name="wrist",
      translation_vector=link4 * scale,
      orientation=[0, 0, 0],
      rotation=rotation4
    ),
    URDFLink(
      name="wrist",
      translation_vector=link5 * scale,
      orientation=[0, 0, 0],
      rotation=rotation5
    )
])

target_position = [0, 0, 1.0]
print("Top position: ", left_arm_chain.inverse_kinematics(geometry_utils.to_transformation_matrix(
    target_position,
    np.eye(3))))
ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')

left_arm_chain.plot(left_arm_chain.inverse_kinematics(geometry_utils.to_transformation_matrix(
    target_position,
    np.eye(3))), ax, target=target_position)
matplotlib.pyplot.show()


target_position = [0.5, 0.5, 0.5]
print("Target position: ", left_arm_chain.inverse_kinematics(geometry_utils.to_transformation_matrix(
    target_position,
    np.eye(3))))
ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')

left_arm_chain.plot(left_arm_chain.inverse_kinematics(geometry_utils.to_transformation_matrix(
    target_position,
    np.eye(3))), ax, target=target_position)
matplotlib.pyplot.show()