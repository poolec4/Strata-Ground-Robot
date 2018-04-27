import serial
import time
import math
import numpy as np
from vicon import Vicon
from robot import Robot
from motion_plan import Planner

TCP_IP = '192.168.10.7'
TCP_PORT = 50000
BUFFER_SIZE = 1024

vicon = Vicon(TCP_IP, TCP_PORT, BUFFER_SIZE)
robot = Robot()

robot.open('/dev/serial0', 19200)
robot.write_motors()

# Trajectory Planning
planner = Planner()
for i in range(10):
	vicon.get_state()
start = [vicon.x_v[0], vicon.x_v[1]]

print('start :', start)

goals = [[-2, -2], [0.3, 0.3]]
x_coords, y_coords, angles, path = planner.planWaypoints(start, goals)
traj_length = len(x_coords[0])
x_traj = x_coords[0]
y_traj = y_coords[0]
th_traj = angles[0]
traj_count = 0

print('path: ', path)
print('angles: ', angles)
print('x_traj: ', x_traj)
print('y_traj: ', y_traj)
print('th_traj: ', th_traj)

time.sleep(2)

t_init = time.time()
t_vicon = 0
t_send = 0

RUN_ROBOT = True

try:
	while RUN_ROBOT:
		print("\n")

		if (time.time() - t_vicon) > 0.1:
			vicon.get_state()
			t_vicon = time.time()

		robot = robot.set_goal([x_traj[traj_count],y_traj[traj_count]], th_traj[traj_count])
		robot = robot.PI_control(vicon.x_v, vicon.q_v, 1-0.0001)

		if (time.time() - t_send) > 0.1:
			robot = robot.write_motors()
			t_send = time.time()

		if robot.p < 0.25:
			traj_count = traj_count + 1
			robot.integral_reset()

		if traj_count == traj_length:
			RUN_ROBOT = False

except KeyboardInterrupt:
	robot.stop_robot()
	print("Robot stopped.. hopefully")
