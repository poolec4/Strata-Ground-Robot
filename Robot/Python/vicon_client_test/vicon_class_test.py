from vicon import Vicon

TCP_IP = '192.168.10.31'
TCP_PORT = 5000
BUFFER_SIZE = 1024

vicon = Vicon(TCP_IP, TCP_PORT, BUFFER_SIZE)

while 1:
	vicon.get_state();
	print("Robot position: " + str(vicon.x_v))
	print("Robot rotation: " + str(vicon.q_v))
