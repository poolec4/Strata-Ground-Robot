# Simple demo of of the PCA9685 PWM servo/LED controller library.
# This will move channel 0 from min to max position repeatedly.
# Author: Tony DiCola
# License: Public Domain
from __future__ import division
import time
import math
import numpy as np

# Import the PCA9685 module.
# import Adafruit_PCA9685

# Import our Controller
from control import controller


# Uncomment to enable debug output.
#import logging
#logging.basicConfig(level=logging.DEBUG)

# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()

# Alternatively specify a different address and/or bus:
#pwm = Adafruit_PCA9685.PCA9685(address=0x41, busnum=2)

# Configure min and max servo pulse lengths
servo_min = 105  # Min pulse length out of 4096
servo_max = 650  # Max pulse length out of 4096

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)

init_angle = 20
servo_angles = [init_angle, init_angle, init_angle, init_angle, init_angle, init_angle] # in degrees
# servo_angles = np.deg2rad(servo_angles) # in radians

ccw = [0,2,4]
cw = [1,3,5]

servo_pulse = np.zeros(6)

# Initialize Controller
controller = controller(init_angle=init_angle, version='v1.0', bounds=(-45, 45))


for i in ccw:
    servo_angles[i] = 270-servo_angles[i]

for i in range(len(servo_angles)):
    servo_pulse[i] = (servo_angles[i])/270*(servo_max-servo_min) + servo_min
    pwm.set_pwm(i, 0, int(round(servo_pulse[i])))


##print('Moving servo on channel 0, press Ctrl-C to quit...')
while True:
    t = int(round(time.time()*1000)) # milliseconds
##    s1 = 135+45*math.sin(2*math.pi*1*t/1000)
##    c1 = 135+45*math.sin(2*math.pi*0.1*t/1000)
##    servo_angles = [s1, c1, s1, c1, s1, c1] # in degrees

##  Controller Usage
## NOTE: Angles must be in radians, time must be in seconds
##  t = time.time # In Seconds
##  controller = controller.step(orientation, translation, t)
##  servo_angles = controller.theta


##    for i in ccw:
##        servo_angles[i] = 270-servo_angles[i]
##
##    for i in range(len(servo_angles)):
##        servo_pulse[i] = (servo_angles[i])/270*(servo_max-servo_min) + servo_min
##        pwm.set_pwm(i, 0, int(round(servo_pulse[i])))

        #print(int(round(servo_pulse[i])))
    time.sleep(0.001)
