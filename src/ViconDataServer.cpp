#include <iostream>
#include <chrono>
#include <thread>
#include <zmq.hpp>  // ZeroMQ for fast communication
#include <Eigen/Dense>  // Eigen for quaternion & angle-axis computation
#include "DataStreamClient.h"  // Vicon SDK

using namespace ViconDataStreamSDK::CPP;
using namespace std::chrono;

int main(int argc, char* argv[]) {
    // Default values
    std::string vicon_ip = "192.168.1.100";  // Default Vicon Tracker IP
    std::string rigid_body_name = "MyRigidBody";  // Default Rigid Body name

    // Parse command-line arguments
    if (argc > 1) vicon_ip = argv[1];  // IP address as first argument
    if (argc > 2) rigid_body_name = argv[2];  // Rigid body name as second argument

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
    socket.bind("tcp://*:5555");  // Bind to port 5555

    // Store previous position and rotation for velocity calculation
    Eigen::Vector3d prev_position(0, 0, 0);
    Eigen::Quaterniond prev_rotation(1, 0, 0, 0);  // Identity quaternion
    double prev_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count() / 1000.0;

    while (true) {
        vicon.GetFrame();
        
        // Get global translation (position)
        Output_GetSegmentGlobalTranslation position = vicon.GetSegmentGlobalTranslation(rigid_body_name, rigid_body_name);
        
        // Get global rotation (quaternion)
        Output_GetSegmentGlobalRotationQuaternion rotation = vicon.GetSegmentGlobalRotationQuaternion(rigid_body_name, rigid_body_name);

        // Get current time
        double current_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count() / 1000.0;
        double dt = current_time - prev_time;

        if (position.Result == Result::Success && rotation.Result == Result::Success && dt > 0) {
            // Convert position to Eigen format
            Eigen::Vector3d current_position(
                position.Translation[0] * 1e-3,  // Convert mm to meters
                position.Translation[1] * 1e-3,
                position.Translation[2] * 1e-3
            );

            // Convert rotation to Eigen quaternion
            Eigen::Quaterniond current_rotation(
                rotation.Rotation[3],  // w
                rotation.Rotation[0],  // x
                rotation.Rotation[1],  // y
                rotation.Rotation[2]   // z
            );

            // Compute translation velocity
            Eigen::Vector3d translation_velocity = (current_position - prev_position) / dt;

            // Compute angular velocity using Angle-Axis representation
            Eigen::AngleAxisd angle_axis_diff(current_rotation.inverse() * prev_rotation);
            Eigen::Vector3d angular_velocity = angle_axis_diff.axis() * angle_axis_diff.angle() / dt;

            // Store previous values
            prev_position = current_position;
            prev_rotation = current_rotation;
            prev_time = current_time;

            // Serialize position, rotation, and velocity data as JSON
            std::stringstream data;
            data << "{ \"position\": { \"translation\": [" 
                 << current_position[0] << "," << current_position[1] << "," << current_position[2] << "],"
                 << "\"rotation\": [" 
                 << current_rotation.w() << "," << current_rotation.x() << ","
                 << current_rotation.y() << "," << current_rotation.z() << "] },"
                 << "\"velocity\": { \"translation\": [" 
                 << translation_velocity[0] << "," << translation_velocity[1] << "," << translation_velocity[2] << "],"
                 << "\"rotation\": [" 
                 << angular_velocity[0] << "," << angular_velocity[1] << "," << angular_velocity[2] << "] } }";

            zmq::message_t message(data.str().size());
            memcpy(message.data(), data.str().c_str(), data.str().size());
            socket.send(message, zmq::send_flags::none);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 100Hz update rate
    }

    return 0;
}

