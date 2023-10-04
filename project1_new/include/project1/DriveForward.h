#ifndef DRIVEFORWARD_H
#define DRIVEFORWARD_H

#include <iostream>
#include <chrono>           // for measuring times

//
#define NOMINAL_LINEAR_VELOCITY 0.2
#define SLOW_LINEAR_VELOCITY 0.1

// Foward declaration
class CWallFollower;

class CDriveForward {
  public:
    // Constructor to get kp, ki, kd
    CDriveForward(double kp, double ki, double kd) 
            : kp_(kp), ki_(ki), kd_(kd), prevError_(0.0), integral_(0.0) {
                prevTime_ = std::chrono::high_resolution_clock::now();
            };


    /**
        * PID calculation member function
        *
        * @param[in] reference
        * @param[in] measured
        * @returns the 
    */ 
    
    double PIDController(double reference, double measured);

    void handler(CWallFollower *wf);

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
