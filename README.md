# Strata-Ground-Robot
GWU Senior Design 2018

Clone repository:
```bash
git clone https://github.com/poolec4/Strata-Ground-Robot.git
cd Strata-Ground-Robot
```

## Platform

To run:
```bash
cd Platform/Python
python main.py
```

## Robot

### Vicon Host (must be run on Linux machine)

##### VRPN
To install VRPN library on the host:
```bash
cd Robot/vicon_host/libraries/vrpn/
```
The Makefile must be edited to match the system type. Uncommemnt the appropriate line (`HW_OS := pc_linux` for 32 bit systems or `HW_OS := pc_linux64` for 64 bit systems):
```bash
nano Makefile
mkdir build
cd build
cmake ..
make
sudo make install
```

##### Host

To build and run the host:
```bash
cd Robot/vicon_host
mkdir build
cd build
cmake ..
make
./base
```
Edit the port for the server by changing `port_number`. This must match the port on the client.

### Rover (on Raspberry Pi)

##### Vicon Client Test
To run test:
```bash
cd Robot/Python/vicon_client_test
python vicon_class_test.py
```

To run main control:
```bash
cd Robot/Python/robot_control
```
Edit the gains of the controller in `robot.py` and the goal in `main.py`:
```bash
nano robot.py
nano main.py
```
Run the program:
```bash
python main.py
```
The robot will drive to the desired goal location and orientation.