
## Parse IMU values in the form "#A=a1,a2,a3#E=y,p,r\n"

import numpy as np
import time

imu_data = "#A=-9.8,1.4,0.4#E=180.2,90.3,33.22212\n"

tic = time.time()

imu_data = imu_data.rstrip()
imu_data = filter(None,imu_data.split('#'))

for i in range(len(imu_data)):
    if "A=" in imu_data[i]:
        print("Parsing accel")
        temp = imu_data[i].replace("A=","")
        temp = np.array(temp.split(','))
        accel = temp.astype(np.float)

    if "E=" in imu_data[i]:
        print("Parsing euler")
        temp = imu_data[i].replace("E=","")
        temp = np.array(temp.split(','))
        e_angles = temp.astype(np.float)

toc = time.time()

t_elapsed = toc-tic

print(t_elapsed)
print(accel)
print(e_angles)