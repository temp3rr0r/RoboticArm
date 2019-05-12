import numpy as np
import matplotlib.pyplot as plt
from sklearn.externals import joblib

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
joblib.dump(cm_to_servo_polynomial_fitter, 'cm_to_servo_polynomial_fitter.sav')

loaded_cm_to_servo_polynomial_fitter = joblib.load('cm_to_servo_polynomial_fitter.sav')
object_side_length = 4.4
open_length = object_side_length * 1.2
open_servo_value = int(loaded_cm_to_servo_polynomial_fitter(open_length))
print("cm: {}, predicted servo value: {}".format(open_length, open_servo_value))

closed_length = object_side_length * 0.8
closed_servo_value = int(loaded_cm_to_servo_polynomial_fitter(closed_length))
print("cm: {}, predicted servo value: {}".format(closed_length, closed_servo_value))


max_closed_length = 6.0
max_closed_servo_value = int(loaded_cm_to_servo_polynomial_fitter(max_closed_length))
print("~ max cm: {}, ~max predicted servo value: {}".format(max_closed_length, max_closed_servo_value))
