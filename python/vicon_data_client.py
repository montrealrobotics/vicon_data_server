import zmq
import json

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://127.0.0.1:5555")  # Replace with C++ server IP
socket.setsockopt_string(zmq.SUBSCRIBE, "")

while True:
    message = socket.recv()
    data = json.loads(message.decode("utf-8"))

    position = data["position"]
    velocity = data["velocity"]

    print(f"Position: {position}")
    print(f"Velocity: {velocity}")

