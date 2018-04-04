import serial
import time
import math
import numpy as np
from robot import robot

# motor vals here are going to be defined as 0 to 255 for forwards and 0 to -255 for backwards
# The conversions will be done in the robot class before writing to arduino


motor_vals = np.asarray([0,0,0,0,0,0])

ardu_ser = serial.Serial('/dev/ttyACM1', 19200)
print(ardu_ser)
buffer = 'A='+str(motor_vals[0])+'&B='+str(-motor_vals[1])+'&C='+str(motor_vals[2])+'&D='+str(-motor_vals[3])+'&E='+str(motor_vals[4])+'&F='+str(-motor_vals[5])+'\n'
ardu_ser.write(buffer)

time.sleep(1.0)
t_init = time.time()

while (time.time()-t_init) < 2.0: 
	motor_vals = 300*np.ones((1,6))
	buffer = 'A='+str(motor_vals[0])+'&B='+str(-motor_vals[1])+'&C='+str(motor_vals[2])+'&D='+str(-motor_vals[3])+'&E='+str(motor_vals[4])+'&F='+str(-motor_vals[5])+'\n'
	ardu_ser.write(buffer)

motor_vals = np.zeros((1,6))
buffer = 'A='+str(motor_vals[0])+'&B='+str(-motor_vals[1])+'&C='+str(motor_vals[2])+'&D='+str(-motor_vals[3])+'&E='+str(motor_vals[4])+'&F='+str(-motor_vals[5])+'\n'
ardu_ser.write(buffer)
