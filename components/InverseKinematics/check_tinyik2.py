import numpy as np
import tinyik


init = np.array([0.0, 0.0, 0.0])
link1 = np.array([0, 0.0, 10.0])
link2 = np.array([0, 0.0, 12.0])
link3 = np.array([0, 0.0, 9.0])
link4 = np.array([0, 0.0, 6.0])
link5 = np.array([0, 0.0, 12.0])

arm = tinyik.Actuator(['z', link1,
                       'y', link2,
                       'y', link3,
                       'y', link4,
                       'z', link5])
print("\nSince the joint angles are zero by default, the end-effector position is at:")
print("arm.angles: ", arm.angles)
print("arm.ee: ", arm.ee)
#
# print("\nSets the joint angles to 30 and 60 degrees to calculate a new position of the end-effector")
# arm.angles = [np.pi / 6, np.pi / 3]  # or np.deg2rad([30, 60])
# print("arm.ee: ", arm.ee)
#
print("\nSets a position of the end-effector to calculate the joint angles")
# arm.ee = [2 / np.sqrt(2), 2 / np.sqrt(2), 0.]
arm.ee = [0.5, 0.5, 0.0]
print("arm.angles: ", arm.angles)
print("rad2 degrees: ", np.round(np.rad2deg(arm.angles)))

# TODO: -1500 to 1500 into degrees
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

k1 = init + link1
k2 = k1 + link2
k3 = k2 + link3
k4 = k3 + link4
k5 = k4 + link5

xyz = np.array([init, k1, k2, k3, k4, k5]).transpose()

# ax.plot(x, y, z, label='parametric curve')
ax.plot(xyz[0], xyz[1], xyz[2], label='parametric curve')
ax.scatter(xyz[0], xyz[1], xyz[2], label='parametric curve')
# plt.title("Target: {}, end: {}".format(target_vector, end))
plt.show()