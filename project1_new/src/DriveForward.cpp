#include "../include/project1/DriveForward.h"
#include "../include/project1/wallFollower.h"
#include <array>
#include <chrono>
#include <cstdlib>


void CDriveForward::handler(CWallFollower* wf)
{
  /*
   * Use -10, -5, 0 , 5, 10 degree points, whatever is smallest use that for
   * PID measured
  */ 
  
  // Determine if the bot should transition to another state wf->nextState
  wf->nextState = wf->States::DRIVE_FOWARD;
  for (const auto &pair : wf->mScanDataRange)
  {
    if (pair.first > 20) // Looks from 20 deg to 180
    {
      // Transition to concave if a point is foind within the bubble size
      if (pair.second < wf->bubble_size_)
      {
        wf->nextState = wf->States::CONCAVE_CORNER;
        break;
      }
    }
    else if(pair.first < 10)  // looks from 5 to -10 deg
    {
      if(pair.second > 2*wf->bubble_size_)
      {
        wf->nextState = wf->States::CONVEX_CORNER;
        break;
      }
    }
  }

  // If the nextState is not drive foward return
  if (wf->nextState != wf->States::DRIVE_FOWARD)
    return;
  
  // Determine if the bot should travel slowly (from 30deg to 180)
  for (const auto &pair : wf->mScanDataRange)
  {
    if(pair.first > 30)
    {
      if(wf->mScanDataRange[pair.first] < 1.5*wf->bubble_size_)
        wf->linearV = SLOW_LINEAR_VELOCITY;
        break;
    }
    wf->linearV = NOMINAL_LINEAR_VELOCITY;
  }

  // Apply PID control
  double smallest_LHS_distance = 100.0;
  for (const auto &pair : wf->mScanDataRange)
  {
    // use -10 to 10 laser points
    if (pair.first < 11)
    {
      smallest_LHS_distance = 
      (pair.second < smallest_LHS_distance) 
      ? pair.second : smallest_LHS_distance;
    }
  }
  wf->angularV = PIDController(wf->bubble_size_, smallest_LHS_distance);
}

double CDriveForward::PIDController(double reference, double measured) 
{
  // Measure the current time to find dt
  auto currentTime = std::chrono::high_resolution_clock::now();

  // Calculate time elapsed since previous iteration
  std::chrono::duration<double> elapsedTime;
  double dt = elapsedTime.count();

  // ensure the dt is not greater than 0.1s (IMPORTANT!)
  dt = (dt > 0.1) ? 0 : dt;
  
  // Calculate error between the reference and the measured signals
  double error = reference - measured;

  // Update the integral term (Euler's method)
  integral_ += error * dt;

  // Calculate the derivative term using backwards difference
  double derivative = (error - prevError_) / dt;

  // Calculate the PID output
  double output = kp_ * error + ki_ * integral_ + kd_ * derivative;

  // Alternatively, just use PI
  // double output = kp_ * error + ki_ * error + kd_;

  // Update the previous time to the current time for next iteration
  prevTime_ = currentTime;

  // Note output may need to be fliped!!!
  // // Bound output 
  // if (output > 1)
  //   output = 1.0;
  // else if (output < -1)
  //   output = -1.0;

  return output;
}
