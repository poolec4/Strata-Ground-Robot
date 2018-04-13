import time
import serial
import math
import numpy as np

class Robot:
	def __init__(self, min_motor_speed=120, max_motor_speed=255, motor_cutoff=5):
		self.motor_vals = np.zeros(6)
		self.min_motor_speed = min_motor_speed
		self.max_motor_speed = max_motor_speed
		self.low_motor_cutoff = motor_cutoff

		self.kp = 3  # kp>0
		self.ka = 40 # kb<0
		self.kb = -20 # ka-kb>0
		self.kpi = 0  # kp>0
		self.kai = 0 # kb<0
		self.kbi = 0 # ka-kb>0

		self.R = 0.0508 # wheel radius in meters
		self.L = 0.3556 # wheelbase width in meters

		self.ardu_ser = None
		self.x_g = None
		self.y_g = None
		self.th_g = None

		self.t_old = time.time()
		self.p_sum = 0
		self.a_sum = 0
		self.b_sum = 0

	def open(self, serial_port, baud_rate):
		self.ardu_ser = serial.Serial(serial_port, baud_rate, timeout=0.1)
		time.sleep(2)
		print(self.ardu_ser)

	def create_trajectory(self, shape, size, num_points):
		if shape == 'circle':
			t = np.linspace(0, 2*math.pi, num_points)
			x = size*np.cos(t)
			y = size*np.sin(t)
		if shape == 'fig8':
			t = np.linspace(0, 2*math.pi, num_points)
			x = size*np.cos(t)
			y = size*np.sin(2*t)
		
		th = np.zeros([num_points])
		for i in range(len(t)-1):
			dx = x[i+1] - x[i]
			dy = y[i+1] - y[i]
			th[i] = math.atan2(dy,dx)

		th[num_points-1] = th[num_points-2]
		traj = [x,y,th]
		return traj

	def set_goal(self, p_g, th_g):
		self.x_g = p_g[0]
		self.y_g = p_g[1]
		self.th_g = th_g
		return self

	def bound_motor_speed(self, v):
		if v > self.max_motor_speed:
			v_bounded = self.max_motor_speed
		elif v < self.min_motor_speed and v > self.low_motor_cutoff:
			v_bounded = self.min_motor_speed
		elif v < -self.max_motor_speed:
			v_bounded = -self.max_motor_speed
		elif v > -self.min_motor_speed and v < -self.low_motor_cutoff:
			v_bounded = -self.min_motor_speed
		else:
			v_bounded = v

		return v_bounded

	def P_control(self, x_v, q_v):
		eangles = quat_to_eangles(q_v)
		th = eangles[2] 
		dx = self.x_g - x_v[0]
		dy = self.y_g - x_v[1]
		print("xv=" + str(x_v[0]) + "yv=" + str(x_v[1]))
		print("dx=" + str(dx) + ", dy=" + str(dy) + ", th=" + str(th))
		
		self.p = math.sqrt(dx**2 + dy**2)
		a = -th + math.atan2(dy,dx)
		b = -th - a + self.th_g

		a = remap_angle(a)
		b = remap_angle(b)

		print("p=" + str(self.p) + ", a=" + str(a) + ", b=" + str(b))

		v = self.kp*self.p
		omega = self.ka*a + self.kb*b

		v_r = (2*v + omega*self.L)/(2*self.R)
		v_l = (2*v - omega*self.L)/(2*self.R)

		v_r = self.bound_motor_speed(v_r)
		v_l = self.bound_motor_speed(v_l)

		print("vl=" + str(v_l) + ", vr=" + str(v_r))

		self.motor_vals[0:3] = v_l
		self.motor_vals[3:6] = v_r
		return self

	def PI_control(self, x_v, q_v, rolloff_fac):

		dt = time.time() - self.t_old

		eangles = quat_to_eangles(q_v)
		th = eangles[2] 
		dx = self.x_g - x_v[0]
		dy = self.y_g - x_v[1]
		print("xv=" + str(x_v[0]) + "yv=" + str(x_v[1]))
		print("dx=" + str(dx) + ", dy=" + str(dy) + ", th=" + str(th))
		
		self.p = math.sqrt(dx**2 + dy**2)
		a = -th + math.atan2(dy,dx)
		b = -th - a + self.th_g

		self.p_sum = rolloff_fac*self.p_sum + self.p*dt
		self.a_sum = rolloff_fac*self.a_sum + a*dt
		self.b_sum = rolloff_fac*self.b_sum + b*dt

		a = remap_angle(a)
		b = remap_angle(b)
		self.a_sum = remap_angle(self.a_sum)
		self.b_sum = remap_angle(self.b_sum)

		print("p=" + str(self.p) + ", a=" + str(a) + ", b=" + str(b))
		print("dt=" + str(dt) + "\np_sum=" + str(self.p_sum) + ", a_sum=" + str(self.a_sum) + ", b_sum=" + str(self.b_sum))

		v = self.kp*self.p + self.kpi*self.p_sum
		omega = self.ka*a + self.kb*b + self.kai*self.a_sum + self.kbi*self.b_sum

		if self.p > 0.05:
			v_r = (2*v + omega*self.L)/(2*self.R)
			v_l = (2*v - omega*self.L)/(2*self.R)
		else:
			v_r = 0
			v_l = 0

		v_r = self.bound_motor_speed(v_r)
		v_l = self.bound_motor_speed(v_l)

		print("vl=" + str(v_l) + ", vr=" + str(v_r))

		self.motor_vals[0:3] = v_l
		self.motor_vals[3:6] = v_r

		self.t_old = time.time()
		return self

	def write_motors(self):
		motor_vals_to_write = np.round(self.motor_vals) + 255
		buffer = 'A='+str(motor_vals_to_write[0])+'&B='+str(motor_vals_to_write[1])+'&C='+str(motor_vals_to_write[2])+'&D='+str(motor_vals_to_write[3])+'&E='+str(motor_vals_to_write[4])+'&F='+str(motor_vals_to_write[5])+"#\n"
		self.ardu_ser.flush()
		self.ardu_ser.write(buffer)
		return self

	def stop_robot(self):
		self.motor_vals = np.zeros(6)
		buffer = "Z#\n"
		self.ardu_ser.write(buffer)
		return self

def quat_to_eangles(quat):
	# quat = [qx qy qz qw]
	qx = quat[0]
	qy = quat[1]
	qz = quat[2]
	qw = quat[3]
	
	phi = math.atan2(2*(qw*qx + qy*qz), 1 - 2*(qx**2 + qy**2))
	theta = math.asin(2*(qw*qy - qz*qx))
	psi = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))

	eangles = [phi, theta, psi] # [roll, pitch, yaw]
	return eangles

def remap_angle(th):
	if th > math.pi:
		th_remap = -(2*math.pi - th)
	elif th < -math.pi:
		th_remap =  2*math.pi + th
	else:
		th_remap = th
	
	return th_remap
