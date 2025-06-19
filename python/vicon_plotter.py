import zmq
import json
import sys
import time
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import numpy as np
import os


def quaternion_to_rpy(qw, qx, qy, qz):
    rotation = R.from_quat([qx, qy, qz, qw])
    return rotation.as_euler('xyz')


def get_unique_filename(base_name="vicon_data", extension=".csv"):
    """
    Generate a unique filename by appending/incrementing a number if the file already exists.
    Args:
        base_name (str): Base name for the file.
        extension (str): File extension (e.g., ".csv").
    Returns:
        str: Unique filename.
    """
    counter = 1
    filename = f"{base_name}{extension}"
    while os.path.exists(filename):
        filename = f"{base_name}{counter}{extension}"
        counter += 1
    return filename


def save_data_to_csv(data, filename):
    """
    Save collected data to a CSV file.
    Args:
        data (list of dict): List of data points collected during the session.
        filename (str): Name of the file to save the data.
    """
    # Convert to a DataFrame and save to CSV
    df = pd.DataFrame(data)
    df.to_csv(filename, index=False)
    print(f"Data saved to {filename}")


def plot_from_csv(csv_filename):
    """
    Plot data from a CSV file.
    Args:
        csv_filename (str): Path to the CSV file.
    """
    # Load data from CSV
    data = pd.read_csv(csv_filename)

    # Extract data
    time_data = data["time"].to_numpy()
    x, y, z = data["x"].to_numpy(), data["y"].to_numpy(), data["z"].to_numpy()
    roll, pitch, yaw = data["roll"].to_numpy(), data["pitch"].to_numpy(), data["yaw"].to_numpy()
    #vx, vy, vz = data["vx"].to_numpy(), data["vy"].to_numpy(), data["vz"].to_numpy()
    #wx, wy, wz = data["wx"].to_numpy(), data["wy"].to_numpy(), data["wz"].to_numpy()

    fig, axs = plt.subplots(4, 1, figsize=(12, 16))

    axs[0].plot(time_data, x, label="X (m)")
    axs[0].plot(time_data, y, label="Y (m)")
    axs[0].plot(time_data, z, label="Z (m)")
    axs[0].set_title("Position")
    axs[0].set_xlabel("Time (s)")
    axs[0].set_ylabel("Position (m)")
    axs[0].legend()
    axs[0].grid(True)

    axs[1].plot(time_data, roll, label="Roll (rad)")
    axs[1].plot(time_data, pitch, label="Pitch (rad)")
    axs[1].plot(time_data, yaw, label="Yaw (rad)")
    axs[1].set_title("Orientation (Roll, Pitch, Yaw)")
    axs[1].set_xlabel("Time (s)")
    axs[1].set_ylabel("Angle (rad)")
    axs[1].legend()
    axs[1].grid(True)

    #axs[2].plot(time_data, vx, label="Vx (m/s)")
    #axs[2].plot(time_data, vy, label="Vy (m/s)")
    #axs[2].plot(time_data, vz, label="Vz (m/s)")
    #axs[2].set_title("Linear Velocity")
    #axs[2].set_xlabel("Time (s)")
    #axs[2].set_ylabel("Linear Velocity (m/s)")
    #axs[2].legend()
    #axs[2].grid(True)

    #axs[3].plot(time_data, wx, label="Wx (rad/s)")
    #axs[3].plot(time_data, wy, label="Wy (rad/s)")
    #axs[3].plot(time_data, wz, label="Wz (rad/s)")
    #axs[3].set_title("Angular Velocity")
    #axs[3].set_xlabel("Time (s)")
    #axs[3].set_ylabel("Angular Velocity (rad/s)")
    #axs[3].legend()
    #axs[3].grid(True)

    plt.tight_layout()
    plt.show()


def plot_live(connection_type):
    """
    Plot live data received over a socket.

    Args:
        connection_type (str): "local" for IPC or "remote" for TCP.
    """
    # ZMQ setup
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
        socket.connect("tcp://127.0.0.1:5555")

    print("Listening for live Vicon data...")

    # Initialize live plotting
    fig, axs = plt.subplots(2, 1, figsize=(12, 16))
    time_data = []
    position_data = []
    rpy_data = []
    #linear_velocity_data = []
    #angular_velocity_data = []
    collected_data = []

    try:
        while True:
            # Receive and decode data
            message = socket.recv()
            data = json.loads(message.decode("utf-8"))

            curr_time = time.time()

            position = data["pose"]["position"]
            orientation = data["pose"]["orientation"]
            #linear_velocity = data["velocity"]["world_frame"]["linear"]
            #angular_velocity = data["velocity"]["world_frame"]["angular"]

            try:
                rpy = quaternion_to_rpy(orientation[3], orientation[0], orientation[1], orientation[2])  # [w, x, y, z]
            except (IndexError, TypeError, ValueError) as e:
                # Handle any errors that might occur (index out of range, wrong type, etc.)
                print(f"Error converting quaternion to rpy: {e}")
                rpy = [0, 0, 0]  # Default to zero values for roll, pitch, yaw
            collected_data.append({
                "time": curr_time,
                "x": position[0], "y": position[1], "z": position[2],
                "roll": rpy[0], "pitch": rpy[1], "yaw": rpy[2]})
                #"vx": linear_velocity[0], "vy": linear_velocity[1], "vz": linear_velocity[2],
                #"wx": angular_velocity[0], "wy": angular_velocity[1], "wz": angular_velocity[2]

            time_data.append(curr_time)
            position_data.append(position)
            #linear_velocity_data.append(linear_velocity)
            #angular_velocity_data.append(angular_velocity)
            rpy_data.append(rpy)

            # Limit data to the last 100 points
            if len(time_data) > 100:
                time_data = time_data[-100:]
                position_data = position_data[-100:]
                rpy_data = rpy_data[-100:]
                #linear_velocity_data = linear_velocity_data[-100:]
                #angular_velocity_data = angular_velocity_data[-100:]

            # Update plots
            axs[0].cla()
            axs[0].plot(time_data, [p[0] for p in position_data], label="X (m)")
            axs[0].plot(time_data, [p[1] for p in position_data], label="Y (m)")
            axs[0].plot(time_data, [p[2] for p in position_data], label="Z (m)")
            axs[0].set_title("Position")
            axs[0].set_xlabel("Time (s)")
            axs[0].set_ylabel("Position (m)")
            axs[0].legend()
            axs[0].grid(True)

            axs[1].cla()
            axs[1].plot(time_data, [r[0] for r in rpy_data], label="Roll (rad)")
            axs[1].plot(time_data, [r[1] for r in rpy_data], label="Pitch (rad)")
            axs[1].plot(time_data, [r[2] for r in rpy_data], label="Yaw (rad)")
            axs[1].set_title("Orientation (Roll, Pitch, Yaw)")
            axs[1].set_xlabel("Time (s)")
            axs[1].set_ylabel("Angle (rad)")
            axs[1].legend()
            axs[1].grid(True)

            #axs[2].cla()
            #axs[2].plot(time_data, [v[0] for v in linear_velocity_data], label="Vx (m/s)")
            #axs[2].plot(time_data, [v[1] for v in linear_velocity_data], label="Vy (m/s)")
            #axs[2].plot(time_data, [v[2] for v in linear_velocity_data], label="Vz (m/s)")
            #axs[2].set_title("Linear Velocity")
            #axs[2].set_xlabel("Time (s)")
            #axs[2].set_ylabel("Linear Velocity (m/s)")
            #axs[2].legend()
            #axs[2].grid(True)

            #axs[3].cla()
            #axs[3].plot(time_data, [w[0] for w in angular_velocity_data], label="Wx (rad/s)")
            #axs[3].plot(time_data, [w[1] for w in angular_velocity_data], label="Wy (rad/s)")
            #axs[3].plot(time_data, [w[2] for w in angular_velocity_data], label="Wz (rad/s)")
            #axs[3].set_title("Angular Velocity")
            #axs[3].set_xlabel("Time (s)")
            #axs[3].set_ylabel("Angular Velocity (rad/s)")
            #axs[3].legend()
            #axs[3].grid(True)

            plt.tight_layout()
            plt.pause(0.01)

    except KeyboardInterrupt:
        print("Live plotting stopped.")

        # Save collected data to CSV
        filename = get_unique_filename("vicon_data", ".csv")
        save_data_to_csv(collected_data, filename)

        plt.close()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python plot_script.py <csv_filename|live> [<local|remote>]")
        sys.exit(1)

    arg = sys.argv[1].lower()

    if arg == "live":
        if len(sys.argv) != 3:
            print("Usage: python plot_script.py live <local|remote>")
            sys.exit(1)

        connection_type = sys.argv[2].lower()
        if connection_type not in ["local", "remote"]:
            print("Invalid argument. Use 'local' for ipc:// or 'remote' for tcp://.")
            sys.exit(1)

        plot_live(connection_type)

    else:
        csv_filename = sys.argv[1]
        plot_from_csv(csv_filename)
