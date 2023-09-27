#include "../include/project1/wallFollower.h"

CWallFollower::CWallFollower()
 : nh_priv_("~")
{
  // Init turtlebot3 node
  ROS_INFO("Wallfollower Simulation Node Init");
  auto ret = init();
  // check initialization
  ROS_ASSERT(ret);
}

CWallFollower::~CWallFollower()
{
  updatecommandVelocity(0.0, 0.0);
  ros::shutdown();
}

bool CWallFollower::init()
{
  // initalisze publishers
  cmd_vel_pub_  = nh_.advertise<geometry_msgs::Twist>( "/cmd_vel", 10 );

  /// initialize subscribers
  // for scan topic
  laser_scan_sub_  = nh_.subscribe("scan", 10, &CWallFollower::laserScanMsgCallBack, this);
  odom_sub_ = nh_.subscribe("odom", 10, &CWallFollower::odomMsgCallBack, this);

  // Initialize varibales

  ROS_INFO("INIT FIN");
  return true;
}

void CWallFollower::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  // // interate over mScanDataRanges
  // for (int num = 0; num < 2; num++)
  // {
  //   // If the scanned distance is inf set to range_max
  //   if (std::isinf(msg->ranges.at(_mScanAngle[num])))
  //   {
  //     mScanDataRanges[num] = msg->range_max;
  //   }
  //   else
  //   {
  //     mScanDataRanges[num] = msg->ranges.at(_mScanAngle[num]);
  //   }
  // }
}

void CWallFollower::updatecommandVelocity(double linear, double angular)
{
  geometry_msgs::Twist cmd_vel;

  cmd_vel.linear.x  = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_.publish(cmd_vel);
}


bool CWallFollower::controlLoop()
{
  /// Implement state machine here!
  // local static variable is set only the first time the function is called.
  static mState currentState = DRIVE_FOWARD;
  static mState nextState;

  switch(currentState)
  {
    case DRIVE_FOWARD:

    break;
    case CONVEX_CORNER:

    break;
    case CONCAVE_CORNER:

    break;
    case FIND_LHS_WALL:

    break;
    default:
      nextState = DRIVE_FOWARD;
    break;
  }
  return true;
}

void CWallFollower::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
  // Calculate the rotation angle
  // Yaw (z-axis rotation)
  double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
	double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
  double yaw = std::atan2(siny, cosy);

  if (yaw < 0)
    yaw = (2*M_PI+yaw);
  
  mRotationAngle = yaw;

}

bool CWallFollower::turnOdom(double angle)
{
  // 
  bool clockWise;

  // we will be sending commands of type "twist"
  geometry_msgs::Twist base_cmd;

  // the command will be to turn at 0.75 rad/s
  base_cmd.linear.x = base_cmd.linear.y = 0.0;
  base_cmd.angular.z = ANGULAR_VELOCITY;

  // Rotates CCW default 
  double angledif = mRotationAngle - angle;
  ros::Rate rate(50.0);
  bool done = false;
  

  while (!done && nh_.ok())
  {
    #ifdef DEBUG
      ROS_INFO("angle [%f] desired angle [%f], CW [%lf]", mRotationAngle*(180/M_PI), angle*(180/M_PI), base_cmd.angular.z);
    #endif
    // update angledif
    angledif = mRotationAngle - angle;

    // want to face a certain direction!
    if( fabs(angledif) > mAngleTolerance)
    {
      if (angledif > 0 && angledif < M_PI)
      {
        base_cmd.angular.z = -ANGULAR_VELOCITY;
      } 
      else if (angledif > 0 && angledif  > M_PI)
      {
        base_cmd.angular.z = ANGULAR_VELOCITY;
      }
      else if (angledif < 0 && -angledif > M_PI)
      {
        base_cmd.angular.z = -ANGULAR_VELOCITY;
      }
      else if (angledif < 0 && -angledif < M_PI)
      {
        base_cmd.angular.z = ANGULAR_VELOCITY;
      }
      else 
      {
        base_cmd.angular.z = ANGULAR_VELOCITY;
      }
      // send the drive command
      cmd_vel_pub_.publish(base_cmd);

      // get callbacks
      ros::spinOnce();
    }
    else {
      base_cmd.angular.z = 0;
      cmd_vel_pub_.publish(base_cmd);
      done = true;
    }
    rate.sleep();
  }
  return true;
}

// ------------------------ Main -----------------------------------------------
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "wallFollower");
  CWallFollower wallFollower;

  ros::Rate loop_rate(50); // 50 Hz

  // ros::ok handles ctrl+C interupts 
  ROS_INFO("Main Whileloop Start");
  
  // Loop until angle is not zero!
  while (ros::ok() && fabs(wallFollower.mRotationAngle) < 1e-2)
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  while (ros::ok())
  {
    wallFollower.controlLoop();
    // callbacks
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}