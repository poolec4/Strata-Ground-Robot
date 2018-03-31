import socket
import time

TCP_IP = '127.0.0.1'
TCP_PORT = 5000
BUFFER_SIZE = 1024

while 1:
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.connect((TCP_IP, TCP_PORT))
	data = s.recv(BUFFER_SIZE)

	print("received data:", data)

	s.close()
	time.sleep(0.1);