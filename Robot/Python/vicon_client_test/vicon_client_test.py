import socket
import time
import numpy as np
from vicon import Vicon

TCP_IP = '127.0.0.1'
TCP_PORT = 5000
BUFFER_SIZE = 1024

x_v = np.zeros((3,1));
q_v = np.zeros((4,1));

while 1:
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.connect((TCP_IP, TCP_PORT))
	data = s.recv(BUFFER_SIZE)
	data = data.rstrip()
	data_b = Vicon.to_str(data)
	data = data_b.split('&')

	for i in range(0,3):
	 	temp = data[i].split('=')
	 	x_v[i] = float(temp[1])
 	
	for i in range(3,7):
		temp = data[i].split('=')
		q_v[i-3] = float(temp[1])

	print(x_v,q_v)
	# print(data)
	s.close()
	time.sleep(0.1);