import time
import math
import numpy as np

# Import our Controller
from control import controller


init_angle = 135.0 # Must be a float

controller = controller(init_angle=init_angle, version='v1.0', bounds=(-45, 45))

orientation = np.asarray([0, 10, 0])
translation = np.asarray([0, 0, 0])

time.sleep(0.01)
t = time.time() # In Seconds
print(t)
controller = controller.step(orientation, translation, t)
print(controller.theta)
