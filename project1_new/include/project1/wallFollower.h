/**
 * Description Here!
 *
 *
 * Looks at one point to the left and one point directly infront
 *
 * Laser Info!
 *  see /opt/ros/noetic/share/sensor_msgs/msg/Range.msg for exact msg
 *  ranges[] should contain 360 elements.
 *  laserScan rotates CCW thus:
 *  ranges[0] is fowards along the x axis of the robot
 *  ranges[90] is the point LHS to the bot.
 *
*/

/*
  Authors: 
  Rohan Meagher 510457860
  ...
  
*/


#ifndef WALLFOLLWER_H
#define WALLFOLLWER_H

// Debug macro
// #define DEBUG

// Directions of the bot for mScanDataRange
#include "DriveForward.h"

#define LINEAR_VELOCITY  0.1
#define ANGULAR_VELOCITY 0.5


// Are all these includes nessisary?
#include "ros/ros.h"
#include <iostream>
#include <string>     
#include <array>
#include <map>                          // for mScanAngles
#include <sensor_msgs/LaserScan.h>      // for laser data
#include <nav_msgs/Odometry.h>          // for odom data
#include <geometry_msgs/Twist.h>        // for geometry msg


// class dstream_out
// {
//   public:
//     //States nextState = DRIVEFORWARD_H;
//     double angularVelocity = 0.0;
//     double linearVelocity = 0.0;
// } ;
class CDriveForward;

class CWallFollower
{
  // ROS NodeHandles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  // ROS Topic Publishers
  ros::Publisher cmd_vel_pub_;

  // ROS Topic Subscribers
  ros::Subscriber laser_scan_sub_;
  ros::Subscriber odom_sub_;

  // Function prototypes
  void updatecommandVelocity(double linear, double angular);
  void laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg);
  void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg);

  // Private Variables
  /**
    * Dictionary that maps a given rotation angle to the angles specified
    * in the laserScan msg. 
  */
  double mAngleTolerance = 0.01;   // pm 0.05 rad
  double mRotationAngle = 0.0;
  std::map<int, int> mRotationAngles{
    {-10, 100},
    {-5, 95},
    {0, 90},
    {5, 85},
    {10, 80},
    {15, 75},
    {20, 70},
    {25, 65},
    {30, 60},
    {35, 55},
    {40, 50},
    {45, 45},
    {50, 40},
    {55, 35},
    {60, 30},
    {65, 25},
    {70, 20},
    {75, 15},
    {80, 10},
    {85, 5},
    {90, 0},
    {95, 355},
    {100, 350},
    {105, 345},
    {110, 340},
    {115, 335},
    {120, 330},
    {125, 325},
    {130, 320},
    {135, 315},
    {140, 310},
    {145, 305},
    {150, 300},
    {155, 295},
    {160, 290},
    {165, 285},
    {170, 280},
    {175, 275},
    {180, 270}
    };
    // FOR PID Controller
    double kp_ = 1.0;
    double ki_ = 1.0;
    double kd_ = 1.0;
  public:
    /// Statemachine enum
    enum class States : int
    {
      DRIVE_FOWARD = 0,
      CONVEX_CORNER,
      CONCAVE_CORNER,
      FIND_LHS_WALL,
      END
    };
    ///
    States currentState = States::DRIVE_FOWARD;
    States nextState;
    ///
    //
    double bubble_size_ = 0.3;
    double bublle_tolerance = 0.05;
    /**
    * Array containing scan range data.
    * Indexes indicate 5 degree increments starting from the LHS of the 
    * Turtlebot e.g. index 0 corresponds to the LHS of the turtleBot,
    * index 1 corresponds to 5 degrees CW, index 18 coresponds to directly
    * infront of the turtlebot and index 36 corresponds to the RHS of the 
    * Turtlebot. 
    * Ranges are in m.
    */
    std::map<int, double> mScanDataRange;

    ///
    double linearV= LINEAR_VELOCITY;
    double angularV = ANGULAR_VELOCITY;

    CWallFollower();
    ~CWallFollower();

    /**
     * Node initialiser function.
     * Initialzes Publishers and Subscribers.
    */
    bool init();

    /**
     * Main Control loop, contains FSM.
    */
    bool controlLoop();

    /**
     * Rotate function
     * 
     * 0/360 deg is down
     * 180 deg is up
     * 90 deg is right
     * 270 deg is left
     * 
     * @param[in] angle angle to be turned to (in rad)
    */
    
    bool turnOdom(double angle);

};

#endif // WALLFOLLWER_H 
