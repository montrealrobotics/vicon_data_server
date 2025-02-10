#include <iostream>
#include <chrono>
#include <thread>
#include <zmq.hpp>
#include <Eigen/Dense>
#include "DataStreamClient.h"

using namespace ViconDataStreamSDK::CPP;
using namespace std::chrono;

int main(int argc, char* argv[]) {
    // Default values
    std::string vicon_ip = "192.168.1.100";
    std::string rigid_body_name = "MyRigidBody";
    std::string socket_location = "local";

    if (argc > 1) vicon_ip = argv[1];
    if (argc > 2) rigid_body_name = argv[2];
    if (argc > 3) socket_location = argv[3];

    std::cout << "Connecting to Vicon Tracker at " << vicon_ip << std::endl;
    std::cout << "Tracking Rigid Body: " << rigid_body_name << std::endl;

    // Connect to Vicon Tracker
    Client vicon;
    if (!vicon.Connect(vicon_ip).Result == Result::Success) {
        std::cerr << "Failed to connect to Vicon Tracker at " << vicon_ip << std::endl;
        return -1;
    }
    vicon.EnableSegmentData();

    // ZeroMQ Publisher
    zmq::context_t context(1);
    zmq::socket_t socket(context, ZMQ_PUB);
    if (socket_location == "local") {
        std::cout << "Using IPC socket, broadcasting to local machine" << std::endl;
        socket.bind("ipc:///tmp/vicon_data");
    } else {
        std::cout << "Using TCP socket, broadcasting to remote machine" << std::endl;
        socket.bind("tcp://*:5555");
    }

    socket.set(zmq::sockopt::sndhwm, 1);

    Eigen::Vector3d prev_position(0, 0, 0);
    Eigen::Quaterniond prev_rotation(1, 0, 0, 0); // Identity quaternion

    using clock = std::chrono::steady_clock;
    using milliseconds = std::chrono::milliseconds;
    int period_ms = 1; // Target 100Hz loop (10ms per iteration)

    while (true) {
        auto loop_start_time = clock::now(); // Start timing the loop

        if (vicon.GetFrame().Result == Result::Success) {
            // Get global translation (position)
            Output_GetSegmentGlobalTranslation position = vicon.GetSegmentGlobalTranslation(rigid_body_name, rigid_body_name);

            // Get global rotation (quaternion)
            Output_GetSegmentGlobalRotationQuaternion rotation = vicon.GetSegmentGlobalRotationQuaternion(rigid_body_name, rigid_body_name);

            double dt = (1. / vicon.GetFrameRate().FrameRateHz);

            if (position.Result == Result::Success && rotation.Result == Result::Success && dt > 0) {
                
                Eigen::Vector3d current_position(
                    position.Translation[0] * 1e-3, // Convert mm to meters
                    position.Translation[1] * 1e-3,
                    position.Translation[2] * 1e-3
                );

                Eigen::Quaterniond current_rotation(
                    rotation.Rotation[3], // w
                    rotation.Rotation[0], // x
                    rotation.Rotation[1], // y
                    rotation.Rotation[2]  // z
                );

                Eigen::Vector3d linear_velocity = (current_position - prev_position) / dt;

                Eigen::AngleAxisd angle_axis_diff(current_rotation.inverse() * prev_rotation);
                Eigen::Vector3d angular_velocity = angle_axis_diff.axis() * angle_axis_diff.angle() / dt;

                prev_position = current_position;
                prev_rotation = current_rotation;

                // Serialize and publish the data
                std::stringstream data;
                data << "{ \"position\": { \"translation\": ["
                     << current_position[0] << "," << current_position[1] << "," << current_position[2] << "],"
                     << "\"rotation\": ["
                     << current_rotation.w() << "," << current_rotation.x() << ","
                     << current_rotation.y() << "," << current_rotation.z() << "] },"
                     << "\"velocity\": { \"linear\": ["
                     << linear_velocity[0] << "," << linear_velocity[1] << "," << linear_velocity[2] << "],"
                     << "\"rotational\": ["
                     << angular_velocity[0] << "," << angular_velocity[1] << "," << angular_velocity[2] << "] } }";

                zmq::message_t message(data.str().size());
                memcpy(message.data(), data.str().c_str(), data.str().size());
                socket.send(message, zmq::send_flags::dontwait);

            }

        }

        // Sleep until the next iteration start time
        auto next_start_time = loop_start_time + milliseconds(period_ms);
        std::this_thread::sleep_until(next_start_time);
    }

    return 0;
}
