# Strata-Ground-Robot
GWU Senior Design 2018

First cone repository:
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

##### Vicon Host (must be run on Linux machine)

To install VRPN library:
```bash
cd Robot/vicon_host/libraries/vrpn/quat
```
The Makefile must be edited to match the system type. Uncommemnt the appropriate line (`HW_OS := pc_linux` for 32 bit systems or `HW_OS := pc_linux64` for 64 bit systems):
```bash
nano Makefile
make
sudo make install
```
Repeat the above procedures, at the vrpn directory, the `vrpn/server_src` directory, and the `vrpn/client_src` directory.

To build and run the host:
```bash
cd Robot/vicon_host
mkdir build
cd build
cmake ..
make
./base
```
##### Rover
