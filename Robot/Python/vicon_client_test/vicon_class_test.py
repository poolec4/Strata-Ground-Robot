from vicon import vicon

TCP_IP = '127.0.0.1'
TCP_PORT = 5000
BUFFER_SIZE = 1024

vicon = vicon(TCP_IP, TCP_PORT, BUFFER_SIZE)

while 1:
	vicon.get_state();
	print("Robot position: " + str(vicon.x_v))
	print("Robot rotation: " + str(vicon.q_v))