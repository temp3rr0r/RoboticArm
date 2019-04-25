from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import matplotlib.pyplot
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from ikpy import geometry_utils

left_arm_chain = Chain(name='left_arm', links=[
    OriginLink(),
    URDFLink(
      name="shoulder",
      translation_vector=[-10, 0, 5],
      orientation=[0, 1.57, 0],
      rotation=[0, 1, 0],
    ),
    URDFLink(
      name="elbow",
      translation_vector=[25, 0, 0],
      orientation=[0, 0, 0],
      rotation=[0, 1, 0],
    ),
    URDFLink(
      name="wrist",
      translation_vector=[22, 0, 0],
      orientation=[0, 0, 0],
      rotation=[0, 1, 0],
    )
])

target_position = [2, 2, 2]
print(left_arm_chain.inverse_kinematics(geometry_utils.to_transformation_matrix(
    target_position,
    np.eye(3))))
ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')

left_arm_chain.plot(left_arm_chain.inverse_kinematics(geometry_utils.to_transformation_matrix(
    [2, 2, 2],
    [[1, 0, 0],
    [0, 1, 0],
    [0, 0, 1]])), ax, target=target_position)
matplotlib.pyplot.show()