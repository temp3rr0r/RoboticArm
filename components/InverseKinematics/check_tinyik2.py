import numpy as np
import tinyik

arm = tinyik.Actuator(['z', [1., 0., 0.], 'z', [1., 0., 0.]])
#Since the joint angles are zero by default, the end-effector position is at (2.0, 0, 0):

print("arm.angles: ", arm.angles)
print("arm.ee: ", arm.ee)
#Sets the joint angles to 30 and 60 degrees to calculate a new position of the end-effector:

arm.angles = [np.pi / 6, np.pi / 3]  # or np.deg2rad([30, 60])
print("arm.ee: ", arm.ee)
#Sets a position of the end-effector to calculate the joint angles:

arm.ee = [2 / np.sqrt(2), 2 / np.sqrt(2), 0.]
print("arm.angles: ", arm.angles)
print("rad2 degrees: ", np.round(np.rad2deg(arm.angles)))