# Main script to control the Strata Ground Robot
# GW SEAS Senior Design 2018
# Written by Chris Poole and Scott Barnes

# Import libraries
from __future__ import division
import serial
import time
import math
import numpy as np
from tools import is_number

# Import our Controller
from control import controller
run_controller = True

# Servo angle definition:
#    Servos [0,2,4] = CW positive rotation with 0 down
#    Servos [1,3,5] = CCW positive rotation with 0 down
# All values sent to Arduino should be **positive** values

init_angle = 135.0 # in degrees
servo_angles = init_angle*np.asarray([1,-1,1,-1,1,-1])

# Init controller
controller = controller(init_angle=init_angle, version='v1.0', bounds=(-45, 45))

# Init Arduino serial connection
ardu_ser = serial.Serial('/dev/ttyACM0', 19200)
print(ardu_ser)

# Init IMU serial connection
imu_ser = serial.Serial('/dev/ttyUSB0', 57600)
print(imu_ser)

# Axis definition (differs from definition printed on the board!):
#   X axis pointing forward (towards the short edge with the connector holes)
#   Y axis pointing to the right (towards reset button)
#   and Z axis pointing down.

# Positive yaw   : clockwise
# Positive roll  : right wing down
# Positive pitch : nose up

# Transformation order: first yaw then pitch then roll.

accel = [0.0 ,0.0, 0.0]
e_angles = [0.0, 0.0, 0.0]
count = 0
first = True

while run_controller == True:
    # time.sleep(0.5)
## Parse IMU values in the form "#A=a1,a2,a3#E=y,p,r\n"
    
##    Traceback (most recent call last):
##  File "main.py", line 62, in <module>
##    accel = temp.astype(np.float)
##ValueError: invalid literal for float(): 237.
    imu_ser.write(str(chr(111)))
    count += 1
    imu_data = imu_ser.readline()
    # print(imu_data)
    imu_data = imu_data.rstrip()
    imu_data = filter(None,imu_data.split('#'))
    
    for i in range(len(imu_data)):
        if "A=" in imu_data[i]:
            temp = imu_data[i].replace("A=","")
            temp = np.array(temp.split(','))
            if (is_number(temp[0]) == True and is_number(temp[1]) == True and is_number(temp[2]) == True):
                accel = np.asarray([float(temp[0]), float(temp[1]), float(temp[2])])
            # else:
                # print(temp)
                # print('Invalid Input')
                #time.delay(1)
            
        if "E=" in imu_data[i]:
            temp = imu_data[i].replace("E=","")
            temp = np.array(temp.split(','))
            if (is_number(temp[0]) == True and is_number(temp[1]) == True and is_number(temp[2]) == True):
                e_angles = np.asarray([float(temp[0]), float(temp[1]), float(temp[2])])
            
            
##Traceback (most recent call last):
##  File "main.py", line 71, in <module>
##    if (is_number(temp[0]) == True and is_number(temp[1]) == True and is_number(temp[2]) == True):
##IndexError: index 2 is out of bounds for axis 0 with size 2

            # else:
                # print(temp)
                # print('Invalid Input')
                # time.delay(1)
    
    # print("Accel = "+str(accel))
    # print("Eangles = "+str(e_angles))
    ## Lookup translation given current servo angles    
    if count > 10000:
        run_controller = False
    if count > 2 and first==True:
        # init_accel = accel
        init_eangles = e_angles
        print('Intial Eangles')
        print(init_eangles)
        first = False
        
    if first == False:
        ##  Controller
        if e_angles[2] > 0:
            e_angles[2] = 180-e_angles[2]
        else:
            e_angles[2] = -(180+e_angles[2])
        # NOTE: Angles must be in degrees, time must be in seconds
        t = time.time() # in seconds
        # Re-Order & Correct Angles
        orientation = np.asarray([-e_angles[1], e_angles[2], -(e_angles[0]-init_eangles[0])]) # whats the order?
        print('Platform Orientation: ')
        print(orientation)
        # controller = controller.step(orientation, np.asarray([0.0, 0.0, 0.0]), t)
        # servo_angles = np.asarray([controller.theta[0], controller.theta[1], controller.theta[2], controller.theta[3], controller.theta[4], controller.theta[5]]) # This will be returned in degrees
        # print('Controller Angles: ')
        # print(controller.theta)
        # print('Servo Angles: ')
        # print(servo_angles)
        ## Set servos via serial buffer to Arduino
        servo_write = servo_angles
        for num in [1,3,5]:
             servo_write[num] = -servo_angles[num] # convert all servo angles to positive (-1*[1,3,5])
        
        ## Test angle definition
        # servo_angles = 135+np.ones(6)*45*np.sin(2*math.pi*1*t)

        servo_write = np.around(servo_write,2)
        buffer = str(servo_write[0])+','+str(servo_write[1])+','+str(servo_write[2])+','+str(servo_write[3])+','+str(servo_write[4])+','+str(servo_write[5])+'\n'
        # ardu_ser.write(buffer)
        # print(buffer)
##Traceback (most recent call last):
##  File "main.py", line 71, in <module>
##  if (is_number(temp[0]) == True and is_number(temp[1]) == True and is_number(temp[2]) == True):
##IndexError: index 2 is out of bounds for axis 0 with size 2

        
ardu_ser.close();
imu_ser.close();
