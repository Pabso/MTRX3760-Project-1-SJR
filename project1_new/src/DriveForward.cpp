#include "DriveForward.h"
#include <chrono>

double CDriveForward::PIDController(double reference, double measured) {
    // Measure the current time to find dt
    auto currentTime = std::chrono::high_resolution_clock::now();

    // Calculate time elapsed since previous iteration
    std::chrono::duration<double> elapsedTime;
    double dt = elapsedTime.count();
    
    // Calculate error between the reference and the measured signals
    double error = reference - measured;

    // Update the integral term (Euler's method)
    integral_ += error * dt;

    // Calculate the derivative term using backwards difference
    double derivative = (error - prevError_) / dt;

    // Calculate the PID output
    double output = kp_ * error + ki_ * error + kd_ * error;

    // Alternatively, just use PI
    // double output = kp_ * error + ki_ * error + kd_;

    // Update the previous time to the current time for next iteration
    prevTime_ = currentTime;

    return output;
}