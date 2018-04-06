import time
import serial
import math
import numpy as np

def quat_to_eangles(quat):
	# quat = [qx qy qz qw]
	
	q1 = quat[0]
	q2 = quat[1]
	q3 = quat[2]
	q0 = quat[3]
	
	phi = math.atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1**2 + q2**2))
	theta = math.asin(2*(q0*q2 - q3*q1))
	psi = math.atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2**2 + q3**2))

	eangles = [phi, theta, psi] # [roll, pitch, yaw]
	return eangles

class Robot:
	def __init__(self, max_motor_speed=255):
		self.motor_vals = np.zeros(6)
		self.max_motor_speed = max_motor_speed

		self.kp = 3  # kp>0
		self.ka = 50 # kb<0
		self.kb = -40 # ka-kb>0
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

	def set_goal(self, p_g, th_g):
		self.x_g = p_g[0]
		self.y_g = p_g[1]
		self.th_g = th_g
		return self

	def P_control(self, x_v, q_v):
		eangles = quat_to_eangles(q_v)
		th = eangles[2] 
		dx = self.x_g - x_v[0]
		dy = self.y_g - x_v[1]
		print("xv=" + str(x_v[0]) + "yv=" + str(x_v[1]))
		print("dx=" + str(dx) + ", dy=" + str(dy) + ", th=" + str(th))
		
		p = math.sqrt(dx**2 + dy**2)
		a = -th + math.atan2(dy,dx)
		b = -th - a + self.th_g

		if a > math.pi:
			a = -(2*math.pi - a)
		if a < -math.pi:
			a =  2*math.pi + a

		if b > math.pi:
			b = -(2*math.pi - b)
		if b < -math.pi:
			b = 2*math.pi + b

		print("p=" + str(p) + ", a=" + str(a) + ", b=" + str(b))

		v = self.kp*p
		omega = self.ka*a + self.kb*b

		v_r = (2*v + omega*self.L)/(2*self.R)
		v_l = (2*v - omega*self.L)/(2*self.R)

		if v_r > 255:
			v_r = self.max_motor_speed

		if v_r < -255:
			v_r = -self.max_motor_speed

		if v_l > 255:
			v_l = self.max_motor_speed

		if v_l < -255:
			v_l = -self.max_motor_speed

		print("vl=" + str(v_l) + ", vr=" + str(v_r))

		self.motor_vals[0:3] = v_l
		self.motor_vals[3:6] = v_r
		return self

	def PI_control(self, x_v, q_v):
		dt = time.time() - t_old

		eangles = quat_to_eangles(q_v)
		th = -eangles[2] 
		dx = self.x_g - x_v[0]
		dy = self.y_g - x_v[1]
		print("xv=" + str(x_v[0]) + "yv=" + str(x_v[1]))
		print("dx=" + str(dx) + ", dy=" + str(dy) + ", th=" + str(th))
		
		p = math.sqrt(dx**2 + dy**2)
		a = -th + math.atan2(dy,dx)
		b = -th - a

		if a > math.pi:
			a = -(2*math.pi - a)
		if a < -math.pi:
			a =  2*math.pi + b

		if b > math.pi:
			b = -(2*math.pi - b)
		if b < -math.pi:
			b = 2*math.pi + b

		print("p=" + str(p) + ", a=" + str(a) + ", b=" + str(b))

		v = self.kp*p
		omega = self.ka*a + self.kb*b

		v_r = (2*v + omega*self.L)/(2*self.R)
		v_l = (2*v - omega*self.L)/(2*self.R)

		if v_r > 255:
			v_r = self.max_motor_speed

		if v_r < -255:
			v_r = -self.max_motor_speed

		if v_l > 255:
			v_l = self.max_motor_speed

		if v_l < -255:
			v_l = -self.max_motor_speed

		print("vl=" + str(v_l) + ", vr=" + str(v_r))

		self.motor_vals[0:3] = v_l
		self.motor_vals[3:6] = v_r

		t_old = time.time
		return self

	def write_motors(self):
		motor_vals_to_write = np.round(self.motor_vals) + 255
		buffer = 'A='+str(motor_vals_to_write[0])+'&B='+str(motor_vals_to_write[1])+'&C='+str(motor_vals_to_write[2])+'&D='+str(motor_vals_to_write[3])+'&E='+str(motor_vals_to_write[4])+'&F='+str(motor_vals_to_write[5])+"#\n"
		self.ardu_ser.flush()
		self.ardu_ser.write(buffer)
		print(buffer)
		return self

	def stop_robot(self):
		self.motor_vals = np.zeros(6)
		buffer = "Z#\n"
		self.ardu_ser.write(buffer)
		return self
