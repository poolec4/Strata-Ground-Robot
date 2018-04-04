import math
import numpy as np

class robot:
	def __init__(self, serial_port, baud_rate):
		ardu_ser = serial.Serial(serial_port, baud_rate)
		print(ardu_ser)
		self.motor_vals = np.array([0,0,0,0,0,0])
		self.kp = 0 # kp>0
		self.ka = 0 # kb<0
		self.kb = 0 # ka-kb>0
		self.R = 0.0508 # wheel radius in meters
		self.L = 0.3556 # wheelbase width in meters

	def set_goal(self, p_g, th_g):
		self.x_g = p_g[1];
		self.y_g = p_g[2];
		self.th_g = th_g;
		return self

	def P_control(self, x_v, q_v):
		eangles = quat_to_eangles(q_v)
		dx = self.x_g - x_v[0];
		dy = self.y_g - x_v[1];
		
		p = math.sqrt(dx**2 + dy**2)
		a = -eangles[0] + math.atan2(dy,dy)
		b = -eangles[0] - a;
		
		s = np.array([p; a; b])
		s_dot = np.array([-kp*p*math.cos(a); kp*math.sin(a) - ka*a - kb*b; -kp*math.sin(a)])
		s = s + s_dot

		v_r = (2*v + omega*self.L)/(2*R)
		v_l = (2*v - omega*self.L)/(2*R)

		motor_vals[0] = v_l
		motor_vals[1] = v_l
		motor_vals[2] = v_l
		motor_vals[3] = v_r
		motor_vals[4] = v_r
		motor_vals[5] = v_r
		return self

	def set_motors(self):
		self.motor_vals = self.motor_vals + 255*np.ones(1,6)
		buffer = 'A='+str(self.motor_vals[0])+'&B='+str(self.motor_vals[1])+'&C='+str(self.motor_vals[2])+'&D='+str(self.motor_vals[3])+'&E='+str(self.motor_vals[4])+'&F='+str(self.motor_vals[5])+'\n'
		self.ardu_ser.write(buffer)
		return self

	def quat_to_eangles(quat)
		alpha = math.atan2(2*(quat[3]*quat[0]-quat[1]*quat[2]), 1-2*(quat[0]^2+quat[2]^2))
		gamma = asin(2*(quat[0]*quat[1]-quat[2]*quat[3]))
		beta = math.atan2(2*(quat[1]*quat[3]-quat[0]*quat[2]),1-2*(quat[1]^2+quat[2]^2))
		eangles = [alpha, beta, gamma];
		return eangles