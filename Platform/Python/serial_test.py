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

# Init IMU serial connection
imu_ser = serial.Serial('/dev/ttyUSB0', 57600)
print(imu_ser)

# Axis definition (differs from definition printed on the board!):
#   X axis pointing forward (towards the short edge with the connector holes)
#   Y axis pointing to the right
#   and Z axis pointing down.

# Positive yaw   : clockwise
# Positive roll  : right wing down
# Positive pitch : nose up

# Transformation order: first yaw then pitch then roll.

accel = [0,0,0]
e_angles = [0,0,0]

while True:
    time.sleep(0.5)
    ## Parse IMU values in the form "#A=a1,a2,a3#E=y,p,r\n"
    #while imu_ser.in_waiting():
    imu_data = imu_ser.readline()    
    print(imu_data)
    
    imu_data = imu_data.rstrip()
    imu_data = filter(None,imu_data.split('#'))

    for i in range(len(imu_data)):
        if "A=" in imu_data[i]:
            temp = imu_data[i].replace("A=","")
            temp = np.array(temp.split(','))
            if (is_number(temp[0]) == True and is_number(temp[1]) == True and is_number(temp[2]) == True):
                accel = np.asarray([float(temp[0]), float(temp[1]), float(temp[2])])
            else:
                print(temp)
                print('Invalid Input')
                exit()
            
        if "E=" in imu_data[i]:
            temp = imu_data[i].replace("E=","")
            temp = np.array(temp.split(','))
            if (is_number(temp[0]) == True and is_number(temp[1]) == True and is_number(temp[2]) == True):
                e_angles = np.asarray([float(temp[0]), float(temp[1]), float(temp[2])])
            else:
                print(temp)
                print('Invalid Input')
                exit()
            
    
    print(accel)
    print(e_angles)
    

imu_ser.close();