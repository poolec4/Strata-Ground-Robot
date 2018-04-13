import serial
import time
import math
import numpy as np
from vicon import Vicon
from robot import Robot

TCP_IP = '192.168.10.7'
TCP_PORT = 50000
BUFFER_SIZE = 1024

x_g = [2, 2]
th_g = 0

vicon = Vicon(TCP_IP, TCP_PORT, BUFFER_SIZE)
robot = Robot()

robot.open('/dev/serial0', 19200)
robot.write_motors()

traj_length = 5
#[x_traj, y_traj, th_traj] = robot.create_trajectory('fig8', 2, traj_length)
traj_count = 0

robot.set_goal(x_g, th_g)

t_init = time.time()
t_vicon = 0
t_send = 0

RUN_ROBOT = True

try:
	while RUN_ROBOT:
		print("\n")
		
		#if (time.time() - t_vicon) > 0.1:
		vicon.get_state()
		t_vicon = time.time()

#		robot = robot.set_goal([x_traj[traj_count],y_traj[traj_count]], th_traj[traj_count])
		robot = robot.PI_control(vicon.x_v, vicon.q_v, 1-0.01)
		
		if (time.time() - t_send) > 0.1:
			robot = robot.write_motors()
			t_send = time.time()

		if robot.p < 0.25:
			traj_count = traj_count + 1
			robot.integral_reset()

		if traj_count == traj_length:
			RUN_ROBOT = False

#		time.sleep(1);

except KeyboardInterrupt:
	robot.stop_robot()
	print("Robot stopped.. hopefully")
