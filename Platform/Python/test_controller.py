import time
import math
import numpy as np

# Import our Controller
from control import controller


init_angle = 135.0 # Must be a float

controller = controller(init_angle=init_angle, version='v1.0')

orientation = np.asarray([10, 10, 0])
translation = np.asarray([80, 50, 10])

time.sleep(0.01)
t0 = time.time() # In Seconds
for i in range(10000):
    controller = controller.step(orientation, translation, t0)
t1 = time.time()
print('Execution Time: ', (t1-t0)/10000.0)
print('Angles: ', controller.theta)
