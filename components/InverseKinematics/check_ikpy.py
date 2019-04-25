from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import numpy as np
import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D
from ikpy import geometry_utils

left_arm_chain = Chain(name='left_arm', links=[
    # OriginLink(),
    URDFLink(
        name="shoulder0",
        # translation_vector=[-10, 0, 5],
        translation_vector=[-0.9, 0.9, -1],
        # orientation=[0, 1.57, 0],
        orientation=[0, 0, 0],
        rotation=[0, 0, 0],
    ),
    URDFLink(
      name="shoulder",
      # translation_vector=[-10, 0, 5],
      translation_vector=[-0.8, 0.5, -1],
      # orientation=[0, 1.57, 0],
      orientation=[0, 0, 0],
      rotation=[0, 0, 0],
    ),
    URDFLink(
      name="elbow",
      # translation_vector=[25, 0, 0],
      translation_vector=[-0.5, 0.5, -1],
      orientation=[0, 0, 0],
      rotation=[0, 0, 0],
    ),
    URDFLink(
      name="wrist",
      # translation_vector=[22, 0, 0],
      translation_vector=[1.0, -0.1, -0.1],
      orientation=[0, 0, 0],
      rotation=[0, 0, 0],
    )
])


positions = left_arm_chain.forward_kinematics([0] * 4)

print("Positions: \n{}".format(positions))

# target_vector = [0.1, -0.2, 0.1]
target_vector = [0.75, -0.75, -1.0]
target_frame = geometry_utils.to_transformation_matrix(
    target_vector,
    np.eye(3))
print("The angles of each joints are : ", left_arm_chain.inverse_kinematics(target_frame))
real_frame = left_arm_chain.forward_kinematics(left_arm_chain.inverse_kinematics(target_frame))
print("Computed position vector : %s, original position vector : %s" % (real_frame[:3, 3], target_frame[:3, 3]))


ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')
end = left_arm_chain.inverse_kinematics(target_frame)
left_arm_chain.plot(end, ax, target=target_vector)

# matplotlib.pyplot.xlim(-0.1, 0.1)
# matplotlib.pyplot.ylim(-0.1, 0.1)
matplotlib.pyplot.title("Target: {}, end: {}".format(target_vector, end))
matplotlib.pyplot.show()