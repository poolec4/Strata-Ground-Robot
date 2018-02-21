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
#    Servos [0,2,4] = CW positive rotation with 0 down
#    Servos [1,3,5] = CCW positive rotation with 0 down
# All values sent to Arduino should be **positive** values

init_angle = 135 # in degrees
servo_angles = init_angle*np.asarray([1,-1,1,-1,1,-1])

# Init controller
controller = controller(init_angle=init_angle, version='v1.0', bounds=(-45, 45))

# Init Arduino serial connection
ardu_ser = serial.Serial('/dev/ttyACM0', 19200)
print(ardu_ser)

# Init IMU serial connection
imu_ser = serial.Serial('/dev/ttyUSB0', 115200)
print(imu_ser)

# Axis definition (differs from definition printed on the board!):
#   X axis pointing forward (towards the short edge with the connector holes)
#   Y axis pointing to the right
#   and Z axis pointing down.

# Positive yaw   : clockwise
# Positive roll  : right wing down
# Positive pitch : nose up

# Transformation order: first yaw then pitch then roll.

time.sleep(2)

while run_controller == True:
    t = time.time() # in seconds
    
    ## Test angle definition
    servo_angles = 135+np.ones(6)*45*np.sin(2*math.pi*1*t)
    
    ## Parse IMU values in the form "#A=a1,a2,a3#E=y,p,r\n"
    while imu_ser.inWaiting():
        imu_data = ser.readline()

    imu_data = imu_data.rstrip()
    imu_data = filter(None,imu_data.split('#'))

    for i in range(len(imu_data)):
        if "A=" in imu_data[i]:
            temp = imu_data[i].replace("A=","")
            temp = np.array(temp.split(','))
            accel = temp.astype(np.float)

        if "E=" in imu_data[i]:
            temp = imu_data[i].replace("E=","")
            temp = np.array(temp.split(','))
            e_angles = temp.astype(np.float)
    
    ## Lookup translation given current servo angles

    ##  Controller 
    # NOTE: Angles must be in radians, time must be in seconds
    controller = controller.step(orientation, translation, t)
    servo_angles = controller.theta # This will be returned in degrees
    

    ## Set servos via serial buffer to Arduino    
    for num in [1,3,5]:
        servo_angles[num] = servo_angles[num] + 360  # convert all servo angles to positive ([1,3,5]+360)
    
    servo_angles = np.around(servo_angles,2)
    buffer = str(servo_angles[0])+','+str(servo_angles[1])+','+str(servo_angles[2])+','+str(servo_angles[3])+','+str(servo_angles[4])+','+str(servo_angles[5])+'\n'
    ardu_ser.write(buffer)
    print(buffer)

ardu_ser.close();
imu_ser.close();