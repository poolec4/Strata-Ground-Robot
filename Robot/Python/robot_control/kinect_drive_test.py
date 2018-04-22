import serial
import time
import math
import freenect
import pdb

import numpy as np
from vicon import Vicon
from kinect import Kinect
from robot import Robot
from kinect2path import plan, World, coordTransform, getLocalGoal, local2global

# from matplotlib import pyplot as plt

TCP_IP = '192.168.10.7'
TCP_PORT = 50000
BUFFER_SIZE = 1024

PLAN_TIME = 100.0;

x_g = [0.0, 1.5] # Goal in global coordinates
th_g = 2.0* math.pi/2.0 # Goal in global coordinates (+CCW)

vicon = Vicon(TCP_IP, TCP_PORT, BUFFER_SIZE)
kinect = Kinect()
robot = Robot()

robot.open('/dev/ttyTHS2', 19200)
robot.write_motors() # Writes initial value (0) to motors

traj_count = 1

t_init = time.time()
t_plan = t_init

world_size = [51,25]
local_start = [int(world_size[0]/2.0), 0]

for i in range(5):
    raw_depth = kinect.get_raw_depth()
    time.sleep(0.1)

for i in range(10):
	vicon.get_state()

RUN_ROBOT = True
FIRST_LOOP = True

# fig1 = plt.figure()
# ax1 = fig1.add_subplot(111)
# fig2 = plt.figure()
# ax2 = fig2.add_subplot(111)
# fig3 = plt.figure()
# ax3 = fig3.add_subplot(111)

try:
	while RUN_ROBOT:
		print("\n")

		## GET VICON DATA
		vicon.get_state()
		print('vicon start position: ', vicon.x_v[0:2])
		print('vicon start angle: ', vicon.th_r)
		t_vicon = time.time()
	
		## GET KINECT DATA AND REPLAN PATH
		if time.time() - t_plan >= PLAN_TIME or FIRST_LOOP == True:
			print("Planning...")
			raw_depth = kinect.get_raw_depth() 
			pcl = kinect.get_point_cloud(raw_depth) # list of points that has to go into path planner
    			depth_map = coordTransform(pcl)
    			world = World(depth_map, world_size=world_size)
    			if world.bounds[1, 0] > -1:
				global_start = np.asarray(vicon.x_v[0:2]).flatten()
    				local_dest = getLocalGoal(global_start, x_g, vicon.th_r, world, local_start)
    				x_coords, y_coords, angles, path, path_cost, world, grid_coords = plan(local_start, local_dest,depth_map, world)
    				x_traj, y_traj, th_traj = local2global(x_coords, y_coords, angles,vicon.x_v[0:2], vicon.th_r, world)
				print('Local Trajectory: ')
				print(x_coords)
				print(y_coords)
				print(angles)
				print('Trajectory:')
				print(x_traj)
				print(y_traj)
				print(th_traj)
				# time.sleep(5)
				traj_length = len(x_traj);
				traj_count = 0;
				t_plan = time.time()
				FIRST_LOOP = False

				# ax1.cla()
        			# ax2.cla()
        			# ax3.cla()
				# ax1.plot(x_coords, y_coords, 'o')
				# ax1.set_xlim([-2, 2])
				# ax1.set_ylim([0, 4])
				# ax2.plot(x_traj, y_traj, 'o')
				# ax2.set_xlim([-3, 3])
				# ax2.set_ylim([-3, 3])
				# ax3.matshow(world.world, cmap='gray')
				# plt.ion()
				# plt.show()
				# plt.pause(10)

		## CONTROL ROBOT
		robot.set_goal([x_traj[traj_count],y_traj[traj_count]], th_traj[traj_count])
		robot = robot.PI_control(vicon.x_v, vicon.q_v, 1)
		robot = robot.write_motors()
		t_send = time.time()		
		print('goal coordinates: ', x_traj[traj_count], y_traj[traj_count], th_traj[traj_count])

		if robot.p < 0.1:
			print('Reached waypoint')
			time.sleep(0.1)
			traj_count = traj_count + 1
	
		robot.integral_reset()

		g_err = math.sqrt((x_g[0]-vicon.x_v[0])**2 + (x_g[1]-vicon.x_v[1])**2)

	 	if g_err < 0.1:
	 		RUN_ROBOT = False

		print("traj_count: " + str(traj_count))
		if traj_count == traj_length:
			print("Reached max traj_count")
			RUN_ROBOT = False

except KeyboardInterrupt:
	robot.stop_robot()
	print("Robot stopped.. hopefully")

