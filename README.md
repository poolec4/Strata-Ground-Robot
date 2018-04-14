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
Edit the gains of the controller in `robot.py` and the goal in `main.py`. Also be sure to change `TCP_IP` and `TCP_PORT` to match the host's IP address (found using `ifconfig`) and the port specified in `vicon_host`:
```bash
nano robot.py
nano main.py
```
Run the program:
```bash
python main.py
```
The robot will drive to the desired goal location and orientation.

##### Kinect
Install libraries:
```bash
sudo apt-get install freenect
sudo apt-get install libfreenect-bin
pip install opencv-python
git clone https://github.com/OpenKinect/libfreenect.git
cd libfreenect/
mkdir build
cd build
cmake ..
make
sudo make install
cd ..
cd wrappers/python
python setup.py -v install
```
Run depth test:
```bash
cd ~/Strata-Ground-Robot/Robot/kinect_test
sudo python raw_depth.py
```
*Note*: this must be run as `sudo` for freenect to access the USB device

Useful information on the kinect can be found here: http://graphics.stanford.edu/~mdfisher/Kinect.html and https://github.com/shiffman/OpenKinect-for-Processing/blob/master/OpenKinect-Processing/examples/Kinect_v1/PointCloud/PointCloud.pde
