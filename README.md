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

## Docker

Current Vicon DataStreamer IP and rigid body name are in the Dockerfile as "172.19.0.61", "go1_v3", edit Dockerfile if you need to update these.

```
sudo docker build -t vicon-dataserver .
sudo docker run --network host vicon-dataserver
```

Then on your local machine, you can run the python client as before. You should only need to install pyzmq with:

```
pip3 install pyzmq
```
