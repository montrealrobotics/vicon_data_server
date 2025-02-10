import zmq
import json
import sys
import time

if len(sys.argv) != 2:
    print("Usage: python client.py <local|remote>")
    sys.exit(1)

connection_type = sys.argv[1].lower()
if connection_type not in ["local", "remote"]:
    print("Invalid argument. Use 'local' for ipc:// or 'remote' for tcp://.")
    sys.exit(1)

context = zmq.Context()
socket = context.socket(zmq.SUB)

if connection_type == "local":
    print("Connecting to local socket (ipc://)...")
    socket.connect("ipc:///tmp/vicon_data")
elif connection_type == "remote":
    print("Connecting to remote socket (tcp://)...")
    # Replace '127.0.0.1' with the actual server IP if needed
    socket.connect("tcp://127.0.0.1:5555")

# Read the newest message
socket.setsockopt(zmq.CONFLATE, 1)
socket.setsockopt_string(zmq.SUBSCRIBE, "")

print("Listening for Vicon data...")

while True:
    message = socket.recv()
    data = json.loads(message.decode("utf-8"))

    position = data["position"]
    velocity = data["velocity"]

    print(f"Position: {position}")
    print(f"Velocity: {velocity}")
