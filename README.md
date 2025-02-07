# Vicon Data Server

This project reads tracking data from Vicon Tracker with the [Datastream SDK](https://www.vicon.com/software/datastream-sdk/), computes linear and angular velocity, and sends it over ZeroMQ for real-time processing. 
Data can be read from python via ZMQ socket.


## Installation

### **Install Dependencies**
Run the install script:
```bash
cd scripts
./install_deps.sh
```

## Build

Run the build script:
```bash
cd scripts
./build.sh
```

## Run the server
```
./ViconDataServer <VICON_IP> <RIGID_BODY_NAME>
```

## Run the python client
```
python3 python/vicon_data_client.py
```


