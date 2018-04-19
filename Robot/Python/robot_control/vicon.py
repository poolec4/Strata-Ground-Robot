import socket
import time
import numpy as np
from robot import quat_to_eangles

def to_bytes(bytes_or_str):
	if isinstance(bytes_or_str, str):
		value = bytes_or_str.encode() # uses 'utf-8' for encoding
	else:
		value = bytes_or_str
	return value # Instance of bytes

def to_str(bytes_or_str):
	if isinstance(bytes_or_str, bytes):
		value = bytes_or_str.decode() # uses 'utf-8' for encoding
	else:
		value = bytes_or_str
	return value # Instance of str

class Vicon:
	def __init__(self, ip_address, port, buffer_size):
		self.IP_ADDR = ip_address
		self.PORT_NUM = port
		self.BUFF_SIZE = buffer_size

		self.x_v = np.zeros((3,1))
		self.q_v = np.zeros((4,1))

		self.th_r = None

	def get_state(self):
		s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		s.connect((self.IP_ADDR, self.PORT_NUM))
		data = s.recv(self.BUFF_SIZE)
		data = data.rstrip()
		data_b = to_str(data)
		data = data_b.split('&')

		for i in range(0,3):
			temp = data[i].split('=')
			self.x_v[i] = float(temp[1])
	 	
		for i in range(3,7):
			temp = data[i].split('=')
			self.q_v[i-3] = float(temp[1])
		s.close()
		self.th_r = quat_to_eangles(self.q_v)[2]
		return self.x_v, self.q_v
