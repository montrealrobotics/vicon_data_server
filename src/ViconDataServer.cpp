#include <iostream>
#include <chrono>
#include <thread>
#include <zmq.hpp>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include "DataStreamClient.h"
#include "ViconDataServer.h"
#include "ViconFrame.h"

using namespace ViconDataStreamSDK::CPP;
using namespace std::chrono;

// Helper function to convert Eigen::Vector3d to JSON array string
std::string to_json_array(const Eigen::Vector3d& vec) {
    std::stringstream ss;
    ss << "[" << vec(0) << ", " << vec(1) << ", " << vec(2) << "]";
    return ss.str();
}

std::string getCmdOption(char** begin, char** end, const std::string& option) {
    char** itr = std::find(begin, end, option);
    if (itr != end && ++itr != end) {
        return std::string(*itr);
    }
    return "";
}

bool cmdOptionExists(char** begin, char** end, const std::string& option) {
    return std::find(begin, end, option) != end;
}

int main(int argc, char* argv[]) {
    // Default values
    std::string vicon_ip = "192.168.1.100";
    std::string rigid_body_name = "Object";
    std::string segment_name = "ObjectSegment";
    std::string socket_location = "local";
    bool track_markers = true;
    int port = 5555;
    int rate = 1000;
    std::string temp;

    temp = getCmdOption(argv, argv + argc, "--ip");
    if (!temp.empty()) vicon_ip = temp;

    temp = getCmdOption(argv, argv + argc, "--object");
    if (!temp.empty()) rigid_body_name = temp;

    temp = getCmdOption(argv, argv + argc, "--connection");
    if (!temp.empty()) socket_location = temp;

    temp = getCmdOption(argv, argv + argc, "--port");
    if (!temp.empty()) port = std::stoi(temp);

    temp = getCmdOption(argv, argv + argc, "--rate");
    if (!temp.empty()) rate = std::stoi(temp);

    if (cmdOptionExists(argv, argv + argc, "--track_markers")) {
        track_markers = true;
    }
    std::cout << "Connecting to Vicon Tracker at " << vicon_ip << std::endl;
    std::cout << "Tracking Rigid Body: " << rigid_body_name << std::endl;

    // Connect to Vicon Tracker
    Client vicon;
    if (!vicon.Connect(vicon_ip).Result == Result::Success) {
        std::cerr << "Failed to connect to Vicon Tracker at " << vicon_ip << std::endl;
        return -1;
    }
    vicon.EnableSegmentData();
    if (track_markers)
    {
        vicon.EnableMarkerData();
    }

    // ZeroMQ Publisher
    zmq::context_t context(1);
    zmq::socket_t socket(context, zmq::socket_type::pub);
    if (socket_location == "local") {
        std::string ipc_path = "ipc:///tmp/vicon_data_" + rigid_body_name;
        socket.bind(ipc_path);
    } else {
        socket.bind("tcp://*:" + port);
    }

    socket.set(zmq::sockopt::sndhwm, 1);

    ViconFrame previous_frame;  // Store data from the previous frame

    using clock = std::chrono::steady_clock;
    using milliseconds = std::chrono::milliseconds;
    int period_ms = static_cast<int>(1000.0 / rate);  // Target 1kHz loop (1ms per iteration)
    double dt = 0;

    vicon.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ClientPull);

    while (vicon.GetFrame().Result != Result::Success) {
        ;
    }

    unsigned subject_count = vicon.GetSubjectCount().SubjectCount;
    bool subject_found = false;
    unsigned marker_count = 0;
    std::vector<std::string> marker_names;

    for (unsigned int i = 0; i < subject_count; ++i)
    {
        // Get the subject name
        std::string subject_name =
            vicon.GetSubjectName(i).SubjectName;

        if (subject_name == rigid_body_name)
        {
            // Taking the first segment as the segment to be tracked
            segment_name = vicon.GetSegmentName(subject_name, 0).SegmentName;
            std::cout << "Tracking object: " << rigid_body_name << ", segment: " << segment_name << std::endl;
            subject_found = true;
            if (track_markers)
            {
                marker_count = vicon.GetMarkerCount(subject_name).MarkerCount;
                std::cout << "Found " << marker_count << " markers for " << rigid_body_name << std::endl;
                // Get all marker names
                for (unsigned int j = 0; j < marker_count; ++j)
                {
                    Output_GetMarkerName marker_name_output = vicon.GetMarkerName(subject_name, j);
                    if (marker_name_output.Result == Result::Success)
                    {
                        marker_names.push_back(marker_name_output.MarkerName);
                        std::cout << "  Marker " << j << ": " << marker_name_output.MarkerName << std::endl;
                    }
                }
            }
            break;
        }
    }

    if (!subject_found) {
        std::cerr << "Error: Could not find rigid body '" << rigid_body_name << "'" << std::endl;
        return -1;
    }

    while (true) {
        auto loop_start_time = clock::now();  // Start timing the loop

        if (vicon.GetFrame().Result == Result::Success) {

            Output_GetFrameNumber frame_number = vicon.GetFrameNumber();

            // Get global translation (position)
            Output_GetSegmentGlobalTranslation position =
                vicon.GetSegmentGlobalTranslation(rigid_body_name, segment_name);

            // Get global rotation (quaternion)
            Output_GetSegmentGlobalRotationQuaternion rotation =
                vicon.GetSegmentGlobalRotationQuaternion(rigid_body_name, segment_name);

            Output_GetObjectQuality tracking_quality = vicon.GetObjectQuality(rigid_body_name);
            double quality = 0.0;
            if( tracking_quality.Result == Result::Success )
            {
              quality = tracking_quality.Quality;
            }else{
              quality = std::nan("");
            }

            if (position.Result == Result::Success && rotation.Result == Result::Success) {

                double frame_rate = vicon.GetFrameRate().FrameRateHz;

                // Update the current frame
                ViconFrame current_frame;
                current_frame.translation_ = Eigen::Vector3d(
                    position.Translation[0] * 1e-3,  // Convert mm to meters
                    position.Translation[1] * 1e-3,
                    position.Translation[2] * 1e-3
                );

                current_frame.rotation_ = Eigen::Quaterniond(
                    rotation.Rotation[3],  // w
                    rotation.Rotation[0],  // x
                    rotation.Rotation[1],  // y
                    rotation.Rotation[2]   // z
                );

                current_frame.frame_number_ = frame_number.FrameNumber;

                if (current_frame.frame_number_ != previous_frame.frame_number_ + 1) {
                    dt = (current_frame.frame_number_ - previous_frame.frame_number_) * (1.0 / frame_rate);
                    frame_rate = 1.0 / dt;
                } else {
                     dt = 1.0 / frame_rate;
                }

                current_frame.velocity_world_frame_.topRows(3) =
                    (current_frame.translation_ - previous_frame.translation_) / dt;

                Eigen::AngleAxisd angle_axis_diff(
                    current_frame.rotation_ * previous_frame.rotation_.inverse());

                current_frame.velocity_world_frame_.bottomRows(3) =
                    angle_axis_diff.axis() * angle_axis_diff.angle() / dt;

                Eigen::Matrix3d R = current_frame.rotation_.inverse().toRotationMatrix();

                current_frame.velocity_body_frame_.topRows(3) =
                    R * current_frame.velocity_world_frame_.topRows(3);

                current_frame.velocity_body_frame_.bottomRows(3) =
                    R * current_frame.velocity_world_frame_.bottomRows(3);

                current_frame.quality_ = quality;
                current_frame.occluded_ = position.Occluded;

                std::stringstream data;

                // Serialize and publish the data
                data << "{"
                << "\"pose\": {"
                << "    \"position\": " << to_json_array(current_frame.translation_) << ","
                << "    \"orientation\": [" << current_frame.rotation_.x() << ", "
                << current_frame.rotation_.y() << ", " << current_frame.rotation_.z() << ", "
                << current_frame.rotation_.w() << "]"
                << "},"
                << "\"velocity\": {"
                << "    \"world_frame\": {"
                << "        \"linear\": " << to_json_array(Eigen::Vector3d(current_frame.velocity_world_frame_.topRows(3))) << ","
                << "        \"angular\": " << to_json_array(Eigen::Vector3d(current_frame.velocity_world_frame_.bottomRows(3))) << ""
                << "    },"
                << "    \"body_frame\": {"
                << "        \"linear\": " << to_json_array(Eigen::Vector3d(current_frame.velocity_body_frame_.topRows(3))) << ","
                << "        \"angular\": " << to_json_array(Eigen::Vector3d(current_frame.velocity_body_frame_.bottomRows(3))) << ""
                << "    }"
                << "},"
                << "\"frame_number\": " << current_frame.frame_number_ << ","
                << "\"frame_rate\": " << frame_rate << ","
                << "\"quality\": " << quality << ","
                << "\"occluded\": " << (current_frame.occluded_ ? "true" : "false");

                // Add marker data if tracking markers
                if (track_markers && !marker_names.empty()) {
                    data << ",\"markers\": [";
                    bool first_marker = true;

                    for (const std::string& marker_name : marker_names) {
                        Output_GetMarkerGlobalTranslation marker_position =
                            vicon.GetMarkerGlobalTranslation(rigid_body_name, marker_name);

                        if (marker_position.Result == Result::Success) {
                            if (!first_marker) {
                                data << ",";
                            }
                            first_marker = false;

                            Eigen::Vector3d marker_pos(
                                marker_position.Translation[0] * 1e-3,  // Convert mm to meters
                                marker_position.Translation[1] * 1e-3,
                                marker_position.Translation[2] * 1e-3
                            );

                            data << "{"
                                 << "\"name\": \"" << marker_name << "\","
                                 << "\"position\": " << to_json_array(marker_pos) << ","
                                 << "\"occluded\": " << (marker_position.Occluded ? "true" : "false")
                                 << "}";
                        }
                    }
                    data << "]";
                }

                data << "}";

                zmq::message_t message(data.str().size());
                memcpy(message.data(), data.str().c_str(), data.str().size());
                socket.send(message, zmq::send_flags::dontwait);

                // Update the previous frame
                previous_frame = current_frame;

            } else {
                std::cerr << "Error: \n"
                << "Position request returned: "
                << ViconDataStreamSDK::CPP::ResultToString(position.Result)
                << "\n"
                << "Rotation request returned: "
                << ViconDataStreamSDK::CPP::ResultToString(rotation.Result)
                << std::endl;
                break;
            }
        }

        // Sleep until the next iteration start time
        auto next_start_time = loop_start_time + milliseconds(period_ms);
        std::this_thread::sleep_until(next_start_time);
    }

    return 0;
}