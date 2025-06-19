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
The server queries the Vicon data streamer SDK at a default rate of 1kHz, the rate that Vicon Tracker is set to will determine the actual rate that data will be received at. We found that connecting to the Vicon machine via ethernet helps to acheive the best update rate.
The server can be run on the same machine as the client or it can be run on a remote machine, if running on a different machine to the client, run with "remote", the default is "local", running from the build directory:

```
./ViconDataServer --ip <VICON_IP> --object <RIGID_BODY_NAME> --loction <SOCKET_LOCATION>
```

The full set of arguments that can be passed are:

ip: The IP of the machine where vicon is running
object: The name of the object that is being tracked
connection: Location of the client, 'remote' or 'local'
port: the port that the data will stream, default 5555
rate: loop rate to fetch frames from vicon
track_markers: if argument included, the position of all markers in the rigid body are tracked, if not, just the rigid body is tracked.

An example:
```
./ViconDataServer --ip 172.19.0.61 --object arena --rate 1 --connection local --port 5553 --track_markers
```

It is possible to run multiple instances of the data server, if you want to track multiple objects.

In one terminal for example:

```
./ViconDataServer --ip 172.19.0.61 --object arena --rate 1
```

In a second terminal:
```
/ViconDataServer --ip 172.19.0.61 --object kamala
```

As these are both streaming to a local client, they bind to "ipc:///tmp/vicon_data_" + rigid_body_name

If using the remote option, different ports should be set for each instance.


## Run the python client
The python client reads position and velocity data from the ViconDataServer. If the server is running on the same machine as the python client, the connecttion is 'local', otherwise 'remote'.
If the server is running in docker, use 'remote'.

```
python3 python/vicon_data_client.py --object kamala --connection local
```
For a remote connection, pass the ip and port:

```
python3 vicon_data_client.py --object arena --port 5553 --connection remote --ip 127.0.0.1
```

vicon_data_client_rb.py does more processing on the data but currently is only implemented for tracking a rigid body and not rigid body + markers.

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
