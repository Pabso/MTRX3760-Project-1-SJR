/**
 * Description Here!
 *
 *
 * Looks at one point to the left and one point directly infront
 *
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
#define LEFT     1
#define CENTER   0



#define LINEAR_VELOCITY  0.1
#define ANGULAR_VELOCITY 0.5

// Are all these includes nessisary?
#include "ros/ros.h"
#include <iostream>
#include <string>     
#include <vector>
#include <sensor_msgs/LaserScan.h>      // for laser data
#include <nav_msgs/Odometry.h>          // for odom data
// #include <tf/transform_listener.h>      // for odom data
#include <geometry_msgs/Twist.h>        // for geometry msg

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

  // We will be listening to TF transforms as well
  //tf::TransformListener listener_;
  

  // Function prototypes
  //void updatecommandVelocity(double linear, double angular);
  void laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg);
  void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg);

  //
  // Statemachine enum
  enum mState 
  {
    DRIVE_FOWARD = 0,
    FIND_LHS_WALL,
    REVERSE,
    LEFT_TURN,
    RIGHT_TURN
    
  };

  // Variables
  double mFowardDistanceLimit;
  double mLeftDistanceLimit;
  double mTolerance;
  double mAngleTolerance;

  /// index zero is the LHS side distance, index 1 is the foward distance.
  double mScanDataRanges[2] = {0.0, 0.0};

  /*
   * ranges[] should contain 360 elements
   * laserScan rotates CCW thus:
   *  ranges[0] is fowards along the x axis of the robot
   *  ranges[90] is the point LHS to the bot.
  */
  const int _mScanAngle[2] = {0, 90};

  void updatecommandVelocity(double linear, double angular);
  void turnLeft();
  void turnRight();
  void snapOrientation();

  public:
    double mRotationAngle; // in rad
    /// Rotate function:
    // 0/360 deg is down
    // 180 deg is up
    // 90 deg is right
    // 270 deg is left
    CWallFollower();
    ~CWallFollower();
    /**
     * Node initialiser function.
     * Initialzes Publishers and Subscribers.
    */
    bool init();
    /**
     * 
    */
    bool controlLoop();
    ///
    bool turnOdom(double angle);

};

#endif // WALLFOLLWER_H 