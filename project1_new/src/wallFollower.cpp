#include "../include/project1/wallFollower.h"
#include "../include/project1/ConcaveCorner.h"
#include "../include/project1/convexState.h"

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
  image_transport::ImageTransport it(nh_);
  image_transport::Publisher pub = it.advertise("camera/image", 10);
  /// initialize subscribers
  // for scan topic
  laser_scan_sub_  = nh_.subscribe("scan", 10, &CWallFollower::laserScanMsgCallBack, this);
  odom_sub_ = nh_.subscribe("odom", 10, &CWallFollower::odomMsgCallBack, this);
  image_transport::Subscriber sub = it.subscribe("camera/image", 10, imageCallback);
  // Initialize varibales

  ROS_INFO("INIT FIN");

  // Fill mScanRanges with zeros
  for ( const auto &pair : mRotationAngles)
    mScanDataRange[pair.first] = 0.0;

  return true;
}


void CWallFollower::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;

  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }

  int total_pixels = cv_ptr->image.cols*cv_ptr->image.rows;
  int red_pixels = 0;

  //Go through all columns
  for(int y = 0; y < cv_ptr->image.cols; y++ )
  { 
    //go through all rows
    for(int x = 0; x < cv_ptr->image.rows; x++ )
    {
      //go through all channels (b,g,r)
      if (cv_ptr->image.data[0 + 3*x + y*cv_ptr->image.cols*4 ] < MAX_BLUE_ && cv_ptr->image.data[1 + 3*x + y*cv_ptr->image.cols*4 ] < MAX_GREEN_ && cv_ptr->image.data[2 + 3*x + y*cv_ptr->image.cols*4 ] > MIN_RED_)
        {
          red_pixels++;
        }
    }
  }

  Density_Red = red_pixels/total_pixels;

}


void CWallFollower::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  for (const auto &pair : mRotationAngles)
  {
    if (std::isinf(msg->ranges.at(pair.second)))
      mScanDataRange[pair.first] = msg->range_max;
    else 
      mScanDataRange[pair.first] = msg->ranges.at(pair.second);
  }
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
  static CDriveForward driveFoward(kp_, ki_, kd_);
  static States currentState = States::DRIVE_FOWARD;
  static States nextState;

  switch(currentState)
  {
    case States::DRIVE_FOWARD:
    {
      driveFoward.handler(this);
      updatecommandVelocity(linearV, angularV);
      break;
    }
    case States::CONVEX_CORNER:
    {
      convexState convexState;
	    convexState.turnLeft(this);
	    //delete convexState;
      break;
    }
    case States::CONCAVE_CORNER:
    {
      CConcaveCorner concaveSolver;
      concaveSolver.handler(this);
      //delete concaveSolver;
      break;
    }
    case States::FIND_LHS_WALL:
    {
      break;
    }
    case States::END:
    {
      break;
    }
    default:
    {  
      nextState = States::DRIVE_FOWARD;
      break;
    }
  }
  // update current State
  currentState = nextState;
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

  ros::Rate loop_rate(30); // 30 Hz

  // ros::ok handles ctrl+C interupts 
  ROS_INFO("Main Whileloop Start");
  
  // // Loop until angle is not zero!
  // while (ros::ok() && fabs(wallFollower.mRotationAngle) < 1e-2)
  // {
  //   ros::spinOnce();
  //   loop_rate.sleep();
  // }

  while (ros::ok())
  {
    wallFollower.controlLoop();
    // callbacks
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
