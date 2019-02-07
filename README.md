## Tactile magnetic sensors YARP module
Very simple module for YARP reading the output of magnetic tactile sensors via CAN and sending them through a YARP port.

### Usage

#### How to build
1. Clone this repository to `$SRC`

2. Execute the following:
```
cd $SRC
mkdir build
cd build
cmake ..
make install
```

#### How to execute
1. Activate the can interface. Within a terminal type:
```
sudo ip link set <interface_name> type can bitrate 1000000                                          
sudo ip link set <interface_name>
```

2. Open the module `Read_magnetic_tactile_sensors` from `YARP manager`.

3. The data is available through the port `/magnetic-tactile-sensors/readings:o` as vector of `N * M * 3` integers (`yarp::sig::Vector<int>`) where `N` is the number of fingers, `M` is the numberof taxel per fingertip and `3` is the size of the output of each vector.

A `RPC` port `/magnetic-tactile-sensors/rpc:i` is available to send command to the `MCU` that reads the data from the sensor.
