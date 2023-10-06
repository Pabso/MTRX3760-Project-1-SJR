#include "../include/project1/wallFollower.h"
#include "../include/project1/ConcaveCorner.h"
#include "../include/project1/convexState.h"
#include <image_transport/image_transport.h>

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
  image_transport::ImageTransport it(nh_);
  cmd_vel_pub_  = nh_.advertise<geometry_msgs::Twist>( "/cmd_vel", 10 );
  image_transport::Publisher pub_img = it.advertise("camera/image", 10);
  /// initialize subscribers
  // for scan topic
  laser_scan_sub_  = nh_.subscribe("scan", 10, &CWallFollower::laserScanMsgCallBack, this);
  odom_sub_ = nh_.subscribe("odom", 10, &CWallFollower::odomMsgCallBack, this);
  image_transport::Subscriber sub_img = it.subscribe("camera/image", 10, &CWallFollower::imageCallback, this);
  // Initialize varibales

  ROS_INFO("INIT FIN");

  // Fill mScanRanges with zeros
  for ( const auto &pair : mRotationAngles)
    mScanDataRange[pair.first] = 0.0;

  return true;
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
void CWallFollower::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  //Convert ROS image to OpenCV to process
  cv_bridge::CvImagePtr cv_ptr;

  try
  {
    //Copy image to pointer in RGB
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }

  //Set inital values
  int total_pixels = cv_ptr->image.cols*cv_ptr->image.rows;
  int red_pixels = 0;

  //Go through all columns
  for(int y = 0; y < cv_ptr->image.cols; y++ )
  { 
    //go through all rows
    for(int x = 0; x < cv_ptr->image.rows; x++ )
    {
      //go through all channels (b,g,r) and determine if in threshold to be a red pixel
      if (cv_ptr->image.data[0 + 3*x + y*cv_ptr->image.cols*4 ] < MAX_BLUE_ && cv_ptr->image.data[1 + 3*x + y*cv_ptr->image.cols*4 ] < MAX_GREEN_ && cv_ptr->image.data[2 + 3*x + y*cv_ptr->image.cols*4 ] > MIN_RED_)
        {
          red_pixels++;
        }
    }
  }

  //Determine RED Desnity
  Density_Red = red_pixels/total_pixels;

  //Display the image using OpenCV
  cv::imshow("Image Processed", cv_ptr->image);
  //Add some delay in miliseconds. The function only works if there is at least one HighGUI window created and the window is active. If there are several HighGUI windows, any of them can be active.
  cv::waitKey(3);
  //Publish new image 
  //pub.publish(cv_ptr->toImageMsg());

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
  CConcaveCorner concaveSolver;
  convexState convexState;
  switch(currentState)
  {
    case States::DRIVE_FOWARD:
      driveFoward.handler(this);
      ROS_INFO("anglular: %f", angularV);
      updatecommandVelocity(linearV, angularV);
    break;
    case States::CONVEX_CORNER:
      ROS_INFO("State [%d]", int(currentState));
      convexState.turnLeft(this);
      updatecommandVelocity(linearV, angularV);
    break;
    case States::CONCAVE_CORNER:
      ROS_INFO("State [%d]", int(currentState));
      concaveSolver.handler(this);
    break;
    case States::FIND_LHS_WALL:
      ROS_INFO("State [%d]", int(currentState));
      nextState = States::FIND_LHS_WALL;
      updatecommandVelocity(0.0, 0.0);
    break;
    case States::END:
    {
      ROS_INFO("State [%d]", int(currentState));
      break;
    }
    default:
      nextState = States::DRIVE_FOWARD;
    break;
  }
  // update current State
  previousState = currentState;
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

  // Turtlebot rotates CCW default 
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

void CWallFollower::debug()
{
  for (const auto &pair : mScanDataRange)
  {
    ROS_INFO("[%d,%f]",pair.first, pair.second);
  }
}

// ------------------------ Main -----------------------------------------------
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "wallFollower");
  CWallFollower wallFollower;

  ros::Rate loop_rate(30); // 30 Hz

  // ros::ok handles ctrl+C interupts 
  
  // Loop until angle is not zero!
  //ROS_INFO("before loop: %d, %f", ros::ok(), (wallFollower.mScanDataRange[90] > 0.05));
  while (ros::ok() && (wallFollower.mScanDataRange[0] < 0.05))
  {
    //std::cout << wallFollower.mScanDataRange[90] << std::endl;
    ros::spinOnce();
    loop_rate.sleep();
    //ROS_INFO("in loop: %f", wallFollower.mScanDataRange[90]);
  }
  //ROS_INFO("Scan data [%f,%f],", wallFollower.mScanDataRange[0], wallFollower.mScanDataRange[180]);
  ROS_INFO("Main Whileloop Start");
  
  while (ros::ok())
  {
    wallFollower.controlLoop();
    //wallFollower.debug();
    // callbacks
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

//--------------------