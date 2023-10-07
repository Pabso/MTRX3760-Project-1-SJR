// ----- Header Guards ----- 
#ifndef WALLFOLLWER_H
#define WALLFOLLWER_H

// -----  Debug macro ----- 
// #define DEBUG

// -----  Defualt cmd_vel defines ----- 
// #include "ConcaveCorner.h"
#define LINEAR_VELOCITY  0.6
#define ANGULAR_VELOCITY 0.3
#define WINDOW_SIZE 2

// ----- Includes ----- 
#include "ros/ros.h"
#include "DriveForward.h" 
#include "ConcaveCorner.h"
#include "convexState.h"           
#include <iostream>
#include <map>                          // for mScanAngles
#include <sensor_msgs/LaserScan.h>      // for laser data
#include <nav_msgs/Odometry.h>          // for odom data
#include <geometry_msgs/Twist.h>        // for geometry msg
#include <std_msgs/Bool.h>              // for camera

// ----- Foward Declarations ----- 
class CDriveForward;
class CConcaveCorner;
class convecState;

class CWallFollower
{
    // ROS NodeHandles
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;

    // ROS Topic Publishers
    ros::Publisher cmd_vel_pub_;

    // ROS Topic Subscribersg
    ros::Subscriber _scan_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber sub_img_bool_;
    ros::Subscriber laser_scan_sub_;

    // ROS callbacks
    void imgBool(const std_msgs::Bool::ConstPtr &msg);
    void updatecommandVelocity(double linear, double angular);
    void laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg);
    void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg);
    // ------- Scan -------
    /**
      * Moving average filter for /scan data
      * Data is filtered using moving average with window size 
      * @see filterWindowSize.
    */
    void applyMovingAverageFilter();
    /**
      * Dictionary that maps a given rotation angle to the angles specified
      * in the laserScan msg. 
    */
    std::map<int, int> mRotationAngles{
    {-90, 180},{-85, 175},{-80, 170},{-75, 165},
    {-70, 160},{-65, 155},{-60, 150},{-55, 145},
    {-50, 140},{-45, 135},{-40, 130},{-35, 125},
    {-30, 120},{-25, 115},{-20, 110},{-15, 105},
    {-10, 100}, {-5, 95}, {0, 90}, {5, 85}, {10, 80}, 
    {15, 75},{20, 70}, {25, 65}, {30, 60}, {35, 55}, 
    {40, 50},{45, 45}, {50, 40}, {55, 35}, {60, 30},
    {65, 25}, {70, 20}, {75, 15}, {80, 10}, {85, 5},
    {90, 0}, {95, 355}, {100, 350}, {105, 345}, {110, 340},
    {115, 335}, {120, 330}, {125, 325}, {130, 320}, 
    {135, 315}, {140, 310}, {145, 305}, {150, 300}, 
    {155, 295}, {160, 290}, {165, 285}, {170, 280},
    {175, 275}, {180, 270}
    };

    // ------- PID coeficients -------
    double kp_ = 2;
    double ki_ = 0.3;
    double kd_ = 0.0;

    // ------- Object Avoidance -------
    /**
     * Radius sizes for object avoidance used 
     * to transition to the convex state, raidus = bubble_size*coeficient 
    */
    const double convex_bubble_size_coeficient = 3;
    const double concave_bubble_size_coeficient = 1.4;

    // ------- Scan Data Filter ------- 
    const int mFilterWindowSize = WINDOW_SIZE;
    std::array<std::map<int, double>, WINDOW_SIZE> movingAverage;

    // ------- TurnOdom ------- 
    double mAngleTolerance = 0.05;   // pm 0.05 rad

  public:
    // ------- State Machine -------
    enum class States : int
    {
      DRIVE_FOWARD = 0,
      CONVEX_CORNER,
      CONCAVE_CORNER,
      FIND_LHS_WALL,
      END
    };
    States currentState = States::DRIVE_FOWARD;
    States previousState = States::DRIVE_FOWARD;
    States nextState;

    // ------ Color Detection -------
    bool detectedRed = false;

    // ------- Object avoidance -------
    double bubble_size_ = 0.155;
    /**
    * Map containing scan raw scan data.
    * Data stored with same key set as @see mRotationAngles
    */
    std::map<int, double> scanDataMapped;
    /**
     * Map containing filtered scan data.
     * Data stored with same key set as @see mRotationAngles
    */
    std::map<int, double> filteredScanData;

    // ------- Rotation Data ------- 
    /// Stores the yaw of the turtleBot
    double mRotationAngle = 0.0;
    
    // ------- Turtlebot velocities ------- 
    double linearV = LINEAR_VELOCITY;
    double angularV = ANGULAR_VELOCITY;
    
    // ------- 
    CWallFollower();
    ~CWallFollower();

    // ------- Public Functions ------- 
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
     * @param[in] angle angle to be turned relatively (in rad), 
     * 0 < angle < 2*pi.
     * From the direction that the turtleBot is currently facing
     * turnOdom will rotate to the given angle relative to its current 
     * facing position, e.g. if angle = pi/4, the turtlebot will rotate 
     * 45 degrees to the right, if angle = 7*pi/4 (315 degrees) the turtlebot
     * will turn 45 degrees to the left.
    */
    bool turnOdom(double angle);

};

#endif // WALLFOLLWER_H 