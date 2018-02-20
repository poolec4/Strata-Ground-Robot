# Main script to control the Strata Ground Robot
# GW SEAS Senior Design 2018
# Written by Chris Poole and Scott Barnes

# Import libraries
from __future__ import division
import serial
import time
import math
import numpy as np

# Import our Controller
from control import controller
run_controller = True

# Servo angle definition:
#    Servos [1,3,5] = CW positive rotation with 0 down.
#    Servos [2,4,6] = CCW positive rotation with 0 down.
# All values sent to Arduino should be **positive** values

init_angle = 135 # in degrees
servo_angles = init_angle*np.asarray([1,-1,1,-1,1,-1])

# Init controller
controller = controller(init_angle=init_angle, version='v1.0', bounds=(-45, 45))

# Init serial connection
ser = serial.Serial('/dev/ttyACM0', 19200)
print(ser)
time.sleep(2)

while run_controller == True:
    t = time.time() # in seconds
    servo_angles = 135+np.ones(6)*45*np.sin(2*math.pi*1*t)
    
    ##  Controller 
    # NOTE: Angles must be in radians, time must be in seconds
    #controller = controller.step(orientation, translation, t)
    #servo_angles = controller.theta # This will be returned in degrees
    
    # convert all servo angles to positive ([2,4,6]+360)
    
    servo_angles = np.around(servo_angles,2)
    buffer = str(servo_angles[0])+','+str(servo_angles[1])+','+str(servo_angles[2])+','+str(servo_angles[3])+','+str(servo_angles[4])+','+str(servo_angles[5])+'\n'
    ser.write(buffer)
    print(buffer)
    time.sleep(0.001)