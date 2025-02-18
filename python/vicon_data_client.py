import zmq
import json
import sys
import time
from vicon_animate import ViconAnimator
import matplotlib.pyplot as plt


if len(sys.argv) != 2:
    print("Usage: python client.py <local|remote>")
    sys.exit(1)

connection_type = sys.argv[1].lower()
if connection_type not in ["local", "remote"]:
    print("Invalid argument. Use 'local' for ipc:// or 'remote' for tcp://.")
    sys.exit(1)

context = zmq.Context()
socket = context.socket(zmq.SUB)
# Read the newest message
socket.setsockopt(zmq.CONFLATE, 1)
socket.setsockopt_string(zmq.SUBSCRIBE, "")

if connection_type == "local":
    print("Connecting to local socket (ipc://)...")
    socket.connect("ipc:///tmp/vicon_data")
elif connection_type == "remote":
    print("Connecting to remote socket (tcp://)...")
    # Replace '127.0.0.1' with the actual server IP if needed
    socket.connect("tcp://127.0.0.1:5555")

print("Listening for Vicon data...")
animator = None
animator = ViconAnimator()

# Receive and decode data
message = socket.recv()

print("Receiving data...")

data = json.loads(message.decode("utf-8"))
prev_frame = data["frame_number"]
frame_rate = data["frame_rate"]

try:
    while True:
        # Receive and decode data
        message = socket.recv()
        data = json.loads(message.decode("utf-8"))

        position = data["pose"]["position"]
        orientation = data["pose"]["orientation"]
        linear_velocity = data["velocity"]["world_frame"]["linear"]
        angular_velocity = data["velocity"]["world_frame"]["angular"]
        curr_frame = data["frame_number"]
        
        dt = (curr_frame - prev_frame) * (1/frame_rate)
        prev_frame = curr_frame

        # Update the animation
        if animator is not None:
            animator.update(position, orientation, linear_velocity, angular_velocity, dt)

        # Allow real-time plotting
        if animator is not None:
            plt.pause(0.01)
        

except KeyboardInterrupt:
    print("Animation stopped.")

# Show the final animation
if animator is not None:
    animator.show()
