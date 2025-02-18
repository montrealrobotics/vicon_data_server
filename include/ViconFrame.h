#include <Eigen/Dense>

class ViconFrame {
public:
    // Position (Translation)
    Eigen::Vector3d translation_ = Eigen::Vector3d::Zero();

    // Rotation (Quaternion)
    Eigen::Quaterniond rotation_ = Eigen::Quaterniond::Identity();

    // Velocities
    Eigen::VectorXd velocity_world_frame_ = Eigen::VectorXd::Zero(6); // 6D: [linear; angular]
    Eigen::VectorXd velocity_body_frame_ = Eigen::VectorXd::Zero(6); // 6D: [linear; angular]

    // Frame number
    int frame_number_ = 0;

    // Tracking quality
    double quality_ = 0.0;

    // Occlusion flag
    bool occluded_ = false;

    // Constructor
    ViconFrame() = default;

    // Reset function (if needed)
    void reset() {
        translation_.setZero();
        rotation_.setIdentity();
        velocity_world_frame_.setZero();
        velocity_body_frame_.setZero();
        frame_number_ = 0;
        quality_ = 0.0;
        occluded_ = false;
    }
};
