// Includes necessary headers
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <cmath> // Include the cmath header for mathematical functions

// Include external dependencies
#include "vehicles/rov/api/RovRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"
#include "common/VectorMath.hpp"
//using namespace msr::airlib;

// Namespace declaration


// Class definition for the ROV control
class RovControl
{
public:
    RovControl(msr::airlib::RovRpcLibClient& client, const Eigen::Vector4d& reference)
        : client_(client), reference_(reference) {}

    // Method to calculate wrench (control signal) based on PID control
    Eigen::Vector4d calculate_wrench()
    {
        // Get current position from client
        Eigen::VectorXd state = getState();

        // Calculate error between reference and actual state
        Eigen::VectorXd error(4);
        error << reference_(0) - state(0), reference_(1) - state(1), reference_(2) - state(2), reference_(3) - state(6);
        
        // Calculate control signal using PID
        Eigen::Vector4d wrench;
        wrench << Kp_ * error(0) + Kd_ * (error(0) - error_t_(0)) + Ki_ * error_i_(0),
            Kp_ * error(1) + Kd_ * (error(1) - error_t_(1)) + Ki_ * error_i_(1),
            Kp_ * error(2) + Kd_ * (error(2) - error_t_(2)) + Ki_ * error_i_(2),
             (Kp_ * error(3) + Kd_ * (error(3) - error_t_(3)) + Ki_ * error_i_(3));


        // Rotate the wrench from world frame to body frame by yaw
        double yaw = state(6); // Assuming state(7) represents yaw angle
        yaw = -yaw;
        // Construct rotation matrix for 3D rotation about the z-axis (yaw)
        Eigen::Matrix3d R;
        R << cos(yaw), sin(yaw), 0,
            -sin(yaw), cos(yaw), 0,
            0, 0, 1;

        // Apply the rotation to the forces (first 3 elements of wrench)
        Eigen::Vector3d forces_world(wrench.head<3>());
        Eigen::Vector3d forces_body = R * forces_world;

        // Update the first 3 elements of the wrench with the rotated forces
        wrench.head<3>() = forces_body;

        // Output debugging information
        std::cout << std::endl
                  << "error_x " << error(0) << std::endl;
        std::cout << std::endl
                  << "error_y " << error(1) << std::endl;
        std::cout << std::endl
                  << "error_z " << error(2) << std::endl;
        std::cout << std::endl
                  << "error_YAW " << error(3) << std::endl;

        //error t-1
        error_t_ = error;

        //add integral error
        error_i_ += error;
        return wrench;
    }

    // Method to retrieve the current state (position and orientation) from the ROV
    Eigen::VectorXd getState()
    {
        auto position = client_.getRovState().getPosition();

        float pitch, roll, yaw;

        // Convert quaternion orientation to Euler angles
        msr::airlib::VectorMathf::toEulerianAngle(client_.getRovState().getOrientation(), pitch, roll, yaw);

        // Construct state vector (position and yaw angle)
        Eigen::VectorXd rov_state(8);
        rov_state << position.x(),
            position.y(),
            -position.z(), // Negate z-axis position to match coordinate convention
            0.0, // Assuming velocity components are not used
            0.0,
            0.0,
            -yaw, // Negative sign to match rotation convention
            0.0;

        return rov_state;
    }

    Eigen::Vector4d error_t_;
    Eigen::Vector4d error_i_;
    Eigen::Vector4d reference_; // Reference vector for position and yaw

private:
    // Proportional gain for PID control
    const double Kp_ = 20.0;
    const double Kd_ = 1.0;
    const double Ki_ = 0.0;
    msr::airlib::RovRpcLibClient& client_; // Reference to ROV RPC client
};

