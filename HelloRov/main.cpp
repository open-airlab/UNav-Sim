/*
    Circular Trajectory Following Controller

    This program implements a controller for an ROV (Remotely Operated Vehicle) to follow a circular trajectory.
    It utilizes the AirSim API for communication with the ROV, and Eigen library for mathematical operations.

    Circular trajectory parameters:
    - Radius of the circle: 1.0
    - Angular velocity: 0.05 radians per second
    - Fixed altitude: 1.0
    - Fixed yaw angle: 0.0

    The controller generates a reference vector for the ROV based on the circular trajectory parameters.
    It calculates the control signal (wrench) using a proportional (P) controller to minimize the error between the reference
    position and the actual ROV state. The control signal is then converted into PWM signals for the motors
    and sent to the ROV for execution.

    Author: Hakim Amer 
    Date: 22/4/2024
*/



#include "vehicles/rov/api/RovRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"
#include "rov_control.hpp"



// Function to generate reference vector for circular trajectory
Eigen::Vector4d generateReference(double time)
{
    // Define circular trajectory parameters
    double radius = 1.0; // Radius of the circle
    double angular_velocity = 0.05; // Angular velocity (radians per second)

    // Compute reference position on the circular trajectory
    double ref_x = radius * cos(angular_velocity * time);
    double ref_y = radius * sin(angular_velocity * time);
    double ref_z = 1.0; // Fixed altitude
    double ref_yaw = 0.0; // Fixed yaw angle

    // Construct and return the reference vector
    return Eigen::Vector4d(ref_x, ref_y, ref_z, ref_yaw);
}





msr::airlib::vector<float> calculatePWM(Eigen::VectorXd wrench)
{
    // Define A matrix
    Eigen::Matrix<double, 4, 8> A;
    A << 0.0, 0.0, 0.0, 0.0, 0.5, 0.5, -0.5, -0.5,     //Thrust allocation matrix of bluerov2 heavy
        0.0, 0.0, 0.0, 0.0, -0.5, 0.5, 0.5, -0.5,
        1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.1, -0.1, 0.1, -0.1;

    std::vector<double> pwm_out(8);


    // Construct vector v from wrench values
    Eigen::Vector4d v;
    v << wrench(0), wrench(1), wrench(2), wrench(3);

    
    Eigen::MatrixXd pwm_vector = (A.transpose() * A).ldlt().solve(A.transpose() * v);

    pwm_vector /= 40.0;  //scale pwm signal

    msr::airlib::vector<float> vect{ static_cast<float>(pwm_vector(0)),
      static_cast<float>(pwm_vector(1)),
       static_cast<float>(pwm_vector(2)),
      static_cast<float>(pwm_vector(3)), static_cast<float>(pwm_vector(4)),
        static_cast<float>(pwm_vector(5)), static_cast<float>(pwm_vector(6)),
        static_cast<float>(pwm_vector(7)) };
        
    //msr::airlib::vector<float> vect{ -0.5f, -0.5f, -0.5f, -0.5f, 0.05f, 0.05f, 0.05f, 0.05f };

    return vect;
}

int main()
{

    msr::airlib::RovRpcLibClient client;
    client.enableApiControl(true);
    client.armDisarm(true);
    client.confirmConnection();
    double time = 0.0; // Initialize time variable

    while (true) {
        // Generate reference vector based on time
        Eigen::Vector4d reference = generateReference(time);

        // Create an instance of RovControl with the generated reference
        RovControl rov_controller(client, reference);

        // Call the calculate_wrench method to get the control signal
        Eigen::Vector4d wrench = rov_controller.calculate_wrench();

        // Calculate PWM signals from the control signal
        msr::airlib::vector<float> pwm = calculatePWM(wrench);

        // Set the duration for which the PWM signals will be applied
        const float duration = 0.01f;

        // Send PWM signals to the ROV asynchronously
        client.moveByMotorPWMsAsync(pwm, duration);

        // Increment time for the next iteration
        time += 0.01; // Adjust as needed based on the desired time step
    }


    return 0;
}
