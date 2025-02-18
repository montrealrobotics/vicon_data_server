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
The server queries the Vicon data streamer SDK at a rate of 1kHz, the rate that Vicon Tracker is set to will determine the actual rate that data will be received at. We found that connecting to the Vicon machine via ethernet helps to acheive the best update rate.
The server can be run on the same machine as the client or it can be run on a remote machine, if running on a different machine to the client, run with "remote" in SOCKET_LOCATION, the default is "local":

```
./ViconDataServer <VICON_IP> <RIGID_BODY_NAME> <SOCKET_LOCATION>
```

## Run the python client
The python client reads position and velocity data from the ViconDataServer. If the server is running on the same machine as the python client. pass the argument 'local', otherwise 'remote'. If the server is running in docker, use 'remote'.

```
python3 python/vicon_data_client.py local
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

## Useful tools

Check out the python folder for some useful(?) scripts.

vicon_plotter.py can be used to plot live data received from the ViconDataStreamer server and save this data to csv or plot from csv file.

vicon_animation.py contains an animator class which can be used to visualise the tracked object while collecting to data for sanity checks. See the vicon_data_client.py for its usage.

If you are trying to find the centroid of your tracked object in vicon, vicon_object_centroid.py is set up to do this for a cuboid body. Our use case is the trunk of the go1.
