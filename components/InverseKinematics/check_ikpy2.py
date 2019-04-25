from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import numpy as np
import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D
from ikpy import geometry_utils

base = np.array([0.0, 0.0, 0.0])
# link1 = np.array([0, 0.0, 10.0])
# link1 = np.array([0, 0.0, 0.1])
link1 = np.array([0, 0.0, 1.0])
# link2 = np.array([0, 0.0, 12.0])
# link2 = np.array([0, 0.0, 0.12])
link2 = np.array([0, 0.0, 1.2])
# link3 = np.array([0, 0.0, 9.0])
# link3 = np.array([0, 0.0, 0.09])
link3 = np.array([0, 0.0, 0.9])
# link4 = np.array([0, 0.0, 6.0])
# link4 = np.array([0, 0.0, 0.06])
link4 = np.array([0, 0.0, 0.6])
# link5 = np.array([0, 0.0, 12.0])
# link5 = np.array([0, 0.0, 0.12])
link5 = np.array([0, 0.0, 1.2])

robotic_arm_chain = Chain(name='robotic_arm', links=[
    # OriginLink(),
    URDFLink(
        name="base",
        # translation_vector=[-10, 0, 5],
        translation_vector=base,
        # orientation=[0, 1.57, 0],
        orientation=[0, 0, 0],
        rotation=[0, 0, 1],
    ),
    URDFLink(
      name="link1",
      # translation_vector=[-10, 0, 5],
      translation_vector=link1,
      # orientation=[0, 1.57, 0],
      orientation=[0, 0, 0],
      rotation=[0, 0, 1],
    ),
    URDFLink(
      name="link2",
      # translation_vector=[25, 0, 0],
      translation_vector=link2,
      orientation=[0, 0, 0],
      rotation=[0, 0, 1],
    ),
    URDFLink(
      name="link3",
      # translation_vector=[22, 0, 0],
      translation_vector=link3,
      orientation=[0, 0, 0],
      rotation=[0, 0, 1],
    )
])


# positions = robotic_arm_chain.forward_kinematics([0] * 4)
# print("Positions: \n{}".format(positions))
#
# # target_vector = [0.1, -0.2, 0.1]
# target_vector = [0.5, 0.5, 0.0]
# target_frame = geometry_utils.to_transformation_matrix(
#     target_vector,
#     np.eye(3))
# print("The angles of each joints are : ", robotic_arm_chain.inverse_kinematics(target_frame))
# real_frame = robotic_arm_chain.forward_kinematics(robotic_arm_chain.inverse_kinematics(target_frame))
# print("Computed position vector : %s, original position vector : %s" % (real_frame[:3, 3], target_frame[:3, 3]))
#
#
# ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')
# end = robotic_arm_chain.inverse_kinematics(target_frame)
# robotic_arm_chain.plot(end, ax, target=target_vector)
#
# # matplotlib.pyplot.xlim(-0.2, 0.2)
# # matplotlib.pyplot.ylim(-0.2, 0.2)
# matplotlib.pyplot.title("Target: {}, end: {}".format(target_vector, end))
# matplotlib.pyplot.show()

ik = robotic_arm_chain.inverse_kinematics([[1, 0, 0, 2],
                             [0, 1, 0, 2],
                             [0, 0, 1, 2],
                             [0, 0, 0, 1]])

print("ik: {}".format(ik))