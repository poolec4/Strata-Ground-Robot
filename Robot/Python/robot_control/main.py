import serial
import time
import math
import numpy as np

motor_val = np.asarray([0,0,0,0,0,0])

ardu_ser = serial.Serial('/dev/ttyACM0', 19200)
print(ardu_ser)
buffer = 'A='+str(motor_val[0])+'&B='+str(-motor_val[1])+'&C='+str(motor_val[2])+'&D='+str(-motor_val[3])+'&E='+str(motor_val[4])+'&F='+str(-motor_val[5])+'\n'
ardu_ser.write(buffer)

time.sleep(1.0)
t_init = time.time()

while (time.time()-t_init) < 10.0: 
	motor_val = 300*np.ones((1,6))
	buffer = 'A='+str(motor_val[0])+'&B='+str(-motor_val[1])+'&C='+str(motor_val[2])+'&D='+str(-motor_val[3])+'&E='+str(motor_val[4])+'&F='+str(-motor_val[5])+'\n'
	ardu_ser.write(buffer)

motor_val = np.zeros((1,6))
buffer = 'A='+str(motor_val[0])+'&B='+str(-motor_val[1])+'&C='+str(motor_val[2])+'&D='+str(-motor_val[3])+'&E='+str(motor_val[4])+'&F='+str(-motor_val[5])+'\n'
ardu_ser.write(buffer)