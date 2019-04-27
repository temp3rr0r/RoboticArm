import numpy as np
import matplotlib.pyplot as plt

opening_cm = np.array([0.1, 1.3, 1.8, 2.3, 3., 3.9, 4.4, 4.8, 5.4, 5.9])
servo_values = np.array([2400, 2300, 2250, 2200, 2100, 2000, 1950, 1850, 1750, 1500])
smoothing_degree = 2

cm_to_servo_polynomial_fitter = np.poly1d(np.polyfit(opening_cm, servo_values, smoothing_degree))
print("polynomial_1d: ", cm_to_servo_polynomial_fitter)

plt.plot(opening_cm, servo_values, "x", label="servo to grip width")
plt.plot(opening_cm, cm_to_servo_polynomial_fitter(opening_cm), label="polynomial 1d (degree: {})".format(smoothing_degree))
plt.ylabel("servo value (absolute)")
plt.xlabel("gripper opening (cm)")
plt.legend(loc='best')
plt.show()


cm1 = 2.0
servo_value1 = int(cm_to_servo_polynomial_fitter(2.0))
print("cm: {}, predicted servo value: {}".format(cm1, servo_value1))
