#include "../include/project1/DriveForward.h"
#include "../include/project1/wallFollower.h"
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <map>
#define DEG_TO_RAD(degrees) ((degrees) * M_PI / 180.0)
// #define DEBUG // debug macro

void CDriveForward::handler(CWallFollower* wf)
{
  // Determine if the bot should transition to another state wf->nextState
  wf->nextState = wf->States::DRIVE_FOWARD;
  int convex_sum = 0;
  int concave_sum = 0;
  for (const auto &pair : wf->filteredScanData)
  {
    // Transition to concave!
    if (pair.first > 85) // Looks from 90 deg to 180
    {
      // Transition to concave if a point is foind within the bubble size
      if (fabs(pair.second) > 1e-2)
      {
        if (pair.second < concave_bubble_size_coeficient*wf->bubble_size_ && pair.second > 1e-2)
        {
          #ifdef DEBUG
            std::cout << "[" << pair.first << "," << pair.second << "]" << std::endl;
          #endif
          concave_sum += 1;
        }
      }
    }
    // Transition to convex!
    if(pair.first > -6 && pair.first < 11)
    {
      if(pair.second > convex_bubble_size_coeficient*wf->bubble_size_)
      {
        #ifdef DEBUG
          std::cout << "[" << pair.first << "," << pair.second << "]" << std::endl;
        #endif
        convex_sum +=1;
      }
    }
    if(pair.first >=-5 && pair.first <= 10)
    {
      if(pair.second > 1.3*wf->bubble_size_ )
      {
        #ifdef DEBUG
          std::cout << "[" << pair.first << "," << pair.second << "]" << std::endl;
        #endif
        convex_sum +=1;
      }
    }
  }
  // if three or more points are within the convex threshold
  if (convex_sum >= 3)
    wf->nextState = wf->States::CONVEX_CORNER;

  // if five or more points are within the concave threshold
  if (concave_sum > 5)
    wf->nextState = wf->States::CONCAVE_CORNER;
  
  // check if maze end is found
  if (wf->detectedRed)
  {
    wf->nextState = wf->States::END;
  }

  // If the nextState is not drive foward return
  if (wf->nextState != wf->States::DRIVE_FOWARD)
  {
    return;
  }
  
  // Determine if the bot should travel slowly (from 85deg to 120)
  for (const auto &pair : wf->filteredScanData)
  {
    if(pair.first > 81 && pair.first < 121)
    {
      if(wf->filteredScanData[pair.first] < 2*wf->bubble_size_)
      {
        wf->linearV = SLOW_LINEAR_VELOCITY;
        break;
      }
    }
    wf->linearV = NOMINAL_LINEAR_VELOCITY;
  }
  double smallestLHSPoint = 100.0;
  int smallestLHSPointAngle = 0;
  // determine smallest LHS point
  for (const auto &pair : wf->filteredScanData)
  {
    if(pair.first > -76 && pair.first < 76)
    {
      //std::cout << "hello" << std::endl;
      // check point is non zero
      if (pair.second > 1e-4 && pair.second < smallestLHSPoint)
      {
        //std::cout << "bruh" << std::endl;
        smallestLHSPoint = pair.second;
        smallestLHSPointAngle = pair.first;
      }
    }
  }
  if (fabs(smallestLHSPoint -100.0) < 1e-4)
  {
    std::cout << "Error, LHS point is zero!" << smallestLHSPoint << std::endl;
  }

  double LHS_average = 0.0;
  int count = 0;
  // take average of n+2, n-2 points
  for (const auto &pair : wf->filteredScanData)
  {
    // check it is non zero
    if (pair.second > 1e-4) 
    {
      if ((pair.first - smallestLHSPointAngle >= -10) && 
          (pair.first - smallestLHSPointAngle <= 10))
      {
        LHS_average += pair.second*std::cos(DEG_TO_RAD(pair.first));
        count++;
      }
      
    }
  }
  LHS_average = LHS_average/count;
  wf->angularV = PIDController(wf->bubble_size_, LHS_average);
}

double CDriveForward::PIDController(double reference, double measured) 
{
  // Measure the current time to find dt
  auto currentTime = std::chrono::high_resolution_clock::now();

  // Calculate time elapsed since previous iteration
  std::chrono::duration<double> elapsedTime;
  double dt = elapsedTime.count();

  // ensure the dt is not greater than 0.1s (IMPORTANT!)
  if (dt > 1)
  {
    integral_ = 0.0;
  }
  
  // Calculate error between the reference and the measured signals
  double error = reference - measured;
  // Update the integral term (Euler's method)
  integral_ += error * dt;

  //Alternatively, just use PI
  double output = kp_ * error + ki_ * error;

  // Update the previous time to the current time for next iteration
  prevTime_ = currentTime;
  if (output > 0)
    {
      output *= 0.5;
    }

  return -output;
}