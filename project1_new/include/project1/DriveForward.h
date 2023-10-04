#ifndef DRIVEFORWARD_H
#define DRIVEFORWARD_H

#include <iostream>
#include <chrono>           // for measuring times

class CDriveForward {
    public:
        // Constructor to get kp, ki, kd
        CDriveForward(double kp, double ki, double kd) 
                : kp_(kp), ki_(ki), kd_(kd), prevError_(0.0), integral_(0.0) {
                    prevTime_ = std::chrono::high_resolution_clock::now();
                };
    

        // PID calculation
        double PIDController(double reference, double measured);

        bool handler();
    private:
        /// ------ Important PID parameters
        double kp_;         // proportional gain
        double ki_;         // integral gain
        double kd_;         // differential gain
        double prevError_;
        double integral_;
        std::chrono::time_point<std::chrono::high_resolution_clock> prevTime_;
        /// ------ Important PID parameters
};

#endif // DRIVEFORWARD_H
