import time
import serial
import math
import numpy as np

def quat_to_eangles(quat):
	alpha = math.atan2(2*(quat[3]*quat[0]-quat[1]*quat[2]), 1-2*(quat[0]**2+quat[2]**2))
	gamma = math.asin(2*(quat[0]*quat[1]-quat[2]*quat[3]))
	beta = math.atan2(2*(quat[1]*quat[3]-quat[0]*quat[2]),1-2*(quat[1]**2+quat[2]**2))
	eangles = [alpha, beta, gamma];
	return eangles

class robot:
	def __init__(self):
		self.motor_vals = np.zeros(6)
		self.kp = 2  # kp>0
		self.ka = 8 # kb<0
		self.kb = -4 # ka-kb>0
		self.R = 0.0508 # wheel radius in meters
		self.L = 0.3556 # wheelbase width in meters

	def open(self, serial_port, baud_rate):
		self.ardu_ser = serial.Serial(serial_port, baud_rate)
		time.sleep(2)
		print(self.ardu_ser)

	def set_goal(self, p_g, th_g):
		self.x_g = p_g[0];
		self.y_g = p_g[1];
		self.th_g = th_g;
		return self

	def P_control(self, x_v, q_v):
		eangles = quat_to_eangles(q_v)
		th = -eangles[2] 
		dx = self.x_g - x_v[0];
		dy = self.y_g - x_v[1];
		print("xv=" + str(x_v[0]) + "yv=" + str(x_v[1]))
		print("dx=" + str(dx) + ", dy=" + str(dy) + ", th=" + str(th))
		
		p = math.sqrt(dx**2 + dy**2)
		a = -th + math.atan2(dy,dx)
		b = -th - a;

		print("p=" + str(p) + ", a=" + str(a) + ", b=" + str(b))
		
		# s = np.array([p; a; b])
		# s_dot = np.array([-kp*p*math.cos(a); kp*math.sin(a) - ka*a - kb*b; -kp*math.sin(a)])
		# s = s + s_dot

		v = self.kp*p
		omega = self.ka*a + self.kb*b

		v_r = (2*v + omega*self.L)/(2*self.R)
		v_l = (2*v - omega*self.L)/(2*self.R)

		if(v_r > 255):
			v_r = 255

		if(v_r < -255):
			v_r = -255

		if(v_l > 255):
			v_l = 255

		if(v_l < -255):
			v_l = -255

		print("vl=" + str(v_l) + ", vr=" + str(v_r))

		self.motor_vals[0:3] = v_l
		self.motor_vals[3:6] = v_r
		return self

	def write_motors(self):
		motor_vals_to_write = self.motor_vals + 255
		buffer = 'A='+str(motor_vals_to_write[0])+'&B='+str(motor_vals_to_write[1])+'&C='+str(motor_vals_to_write[2])+'&D='+str(motor_vals_to_write[3])+'&E='+str(motor_vals_to_write[4])+'&F='+str(motor_vals_to_write[5])+'\n'
		self.ardu_ser.write(buffer)
		#print(buffer)
		return self

	def stop_robot(self):
		self.motor_vals = np.zeros(6)
		self.write_motors()
