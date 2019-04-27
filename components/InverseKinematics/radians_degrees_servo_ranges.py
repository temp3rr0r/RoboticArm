import numpy as np
from sklearn.preprocessing import minmax_scale
# TODO: 0 degrees = 1500. -180 degrees = 500, +180 degrees = 2500


def servo_to_angle_range(values):
    return minmax_scale(values, feature_range=(-180/2.0, 180/2.0))


def angle_to_servo_range(values):
    return minmax_scale(values, feature_range=(500, 2500))


def radians_to_servo_range_unrounded(x, x_min=(-np.pi / 2.0), x_max=(np.pi / 2.0), scaled_min=500.0, scaled_max=2500.0):
    x_std = (x - x_min) / (x_max - x_min)
    return x_std * (scaled_max - scaled_min) + scaled_min


def radians_to_servo_range(x, x_min=(-np.pi / 2.0), x_max=(np.pi / 2.0), scaled_min=500.0, scaled_max=2500.0):
    x_std = (x - x_min) / (x_max - x_min)
    return np.round(x_std * (scaled_max - scaled_min) + scaled_min, 0).astype(int)

values1 = np.array([1500.0, 500, 2500, 1300])
# audio_scaled = minmax_scale(audio, feature_range=(-1,1))
# shape = image.shape
# image_scaled = minmax_scale(image.ravel(), feature_range=(0,255)).reshape(shape)
print("servo: {} -> angle: {}".format(values1, servo_to_angle_range(values1)))

values2 = np.array([0.0, -90, 90.0, 45.0])
print("angle: {} -> servo: {}".format(values2, angle_to_servo_range(values2)))

print("angle: {} -> radians: {}".format(-45, np.deg2rad(-45.0)))

# values3 = np.array([0.0, -np.pi * 2, np.pi * 2, np.pi/4.0, -np.pi, np.pi])
values3a = np.array([0.0, -90, 90.0, 45.0])
values3 = np.deg2rad(values3a)
print("angle: {} -> radians: {}".format(values3a, values3))

print("radians: {} -> servo: {}".format(values3, radians_to_servo_range_unrounded(values3)))


print("radians: {} -> servo(rounded): {}".format(values3, radians_to_servo_range(values3)))