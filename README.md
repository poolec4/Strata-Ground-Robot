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

##### GRPC

To install GRPC for C++, build it from the source:
```bash
sudo apt-get install build-essential autoconf libtool pkg-config
sudo apt-get install autoconf
git clone -b $(curl -L https://grpc.io/release) https://github.com/grpc/grpc
cd grpc/third_party/protobuf
sudo make install
cd ../..
git submodule update --init
sudo make HAS_SYSTEM_PROTOBUF=false
sudo make install
```
Try compiling the GRPC examples:
```bash
cd examples/cpp/hellowworld
make
```
Run the server and client (in different terminal windows):
```bash
./greeter_server
./greeter_client
```
The client window should print `Greeter received: Hello world` if everything is working.

To install GRPC for Python:
```bash
python -m pip install grpcio
python -m pip install grpcio-tools
```
Try GRPC examples (once again in separate terminal windows):
```bash
git clone -b v1.10.x https://github.com/grpc/grpc
cd grpc/examples/python/helloworld
python greeter_server.py
python greeter_client.py
```
The client window should print `Greeter client received: Hello, you!` if everything is working.

Next, test sending from one device to another. Might need to do this: https://github.com/grpc/grpc/issues/10946

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
### Rover (on Raspberry Pi)
