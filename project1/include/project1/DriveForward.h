#ifndef DRIVEFORWARD_H
#define DRIVEFORWARD_H

#include <iostream>
#include <chrono>           // for measuring times
#include <map>

// When a wall is not close
#define NOMINAL_LINEAR_VELOCITY 0.06
// When a wall is close
#define SLOW_LINEAR_VELOCITY    0.03

// Foward declaration
class CWallFollower;

/**
 * 
 * CDriveForward uses a PI controller to maintain a distance from the wall.
 * It calculates the error by taking the difference between the LiDAR 
 * distance measurement and the threshold distance it wants to be at.
 * It calculates the measured distance by taking the minimum distance to
 * the left of the robot within a range, then fanning outwards and averaging
 * the cosines of the adjascent measurements to get a more robust averaged
 * measurement. The PI then adjusts its course to move closer or further
 * away from the wall.
 * 
 * It interfaces with CWallFollower to obtain the LiDAR data, and to compute
 * the moving average.
 * 
*/
class CDriveForward {
  public:
    // Constructor to get kp, ki, kd
    CDriveForward(double kp, double ki, double kd) 
            : kp_(kp), ki_(ki), kd_(kd), prevError_(0.0), integral_(0.0) {
                prevTime_ = std::chrono::high_resolution_clock::now();
            };
    void handler(CWallFollower *wf);

  private:
    // How large the convex bubble size should be for the concave and convex states
    const double convex_bubble_size_coeficient = 5;
    const double concave_bubble_size_coeficient = 1.25;
    /**
      * PID calculation member function
      *
      * @param[in] reference
      * @param[in] measured
      * @returns angular velocity
    */ 
    double PIDController(double reference, double measured);
    /// ------ Important PID parameters
    double kp_;         // proportional gain
    double ki_;         // integral gain
    double kd_;         // differential gain
    double prevError_;
    double integral_;
    /// ------
    std::chrono::time_point<std::chrono::high_resolution_clock> prevTime_;
};

#endif // DRIVEFORWARD_H
