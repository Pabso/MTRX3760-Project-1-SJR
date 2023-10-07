#include "../include/project1/wallFollower.h"
// #include "../include/project1/ConcaveCorner.h"
// #include "../include/project1/convexState.h"


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
  //image_transport::ImageTransport it(nh_);
  cmd_vel_pub_  = nh_.advertise<geometry_msgs::Twist>( "/cmd_vel", 10 );
  //image_transport::Publisher pub_img = it.advertise("camera/image", 10);
  //image_transport::ImageTransport it(nh_);
  /// initialize subscribers
  laser_scan_sub_  = nh_.subscribe("scan", 10, &CWallFollower::laserScanMsgCallBack, this);
  odom_sub_ = nh_.subscribe("odom", 10, &CWallFollower::odomMsgCallBack, this);
  sub_img_bool_ = nh_.subscribe("Red_bool", 10, &CWallFollower::imgBool, this);
  //image_transport::Subscriber sub = it.subscribe("/camera/image_raw", 10, &CWallFollower::imageCallback, this);
  //sub_img = nh_.subscribe("/camera/image_raw/compressed", 2, &CWallFollower::imageCallback, this);

  // Fill mScanRanges with zeros
  for ( const auto &pair : mRotationAngles)
    scanDataMapped[pair.first] = 0.0;

  return true;
}

void CWallFollower::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  for (const auto &pair : mRotationAngles)
  {
    if (std::isinf(msg->ranges.at(pair.second)))
      scanDataMapped[pair.first] = msg->range_max;
    else 
      scanDataMapped[pair.first] = msg->ranges.at(pair.second);

  }
  // Fill in moving average filter 
  applyMovingAverageFilter();
}

void CWallFollower::imgBool(const std_msgs::Bool::ConstPtr &msg)
{
  detectedRed = msg->data;
}

// void CWallFollower::imageCallback(const sensor_msgs::ImageConstPtr& msg)
// {
//   std::cout << "In img callback" << std::endl;
//   // //Convert ROS image to OpenCV to process
//   // cv_bridge::CvImagePtr cv_ptr;

//   // try
//   // {
//   //   //Copy image to pointer in RGB
//   //   cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//   // }
//   // catch (cv_bridge::Exception& e)
//   // {
//   //   ROS_ERROR("Sup");
//   //   //ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
//   // }

//   // //Set inital values
//   // int total_pixels = cv_ptr->image.cols*cv_ptr->image.rows;
//   // int red_pixels = 0;

//   // //Go through all columns
//   // for(int y = 0; y < cv_ptr->image.cols; y++ )
//   // { 
//   //   //go through all rows
//   //   for(int x = 0; x < cv_ptr->image.rows; x++ )
//   //   {
//   //     //go through all channels (b,g,r) and determine if in threshold to be a red pixel
//   //     if (cv_ptr->image.data[0 + 3*x + y*cv_ptr->image.cols*4 ] < MAX_BLUE_ && cv_ptr->image.data[1 + 3*x + y*cv_ptr->image.cols*4 ] < MAX_GREEN_ && cv_ptr->image.data[2 + 3*x + y*cv_ptr->image.cols*4 ] > MIN_RED_)
//   //       {
//   //         red_pixels++;
//   //       }
//   //   }
//   // }

//   // //Determine RED Desnity
//   // Density_Red = red_pixels/total_pixels;

//   // //Display the image using OpenCV
//   // cv::imshow("Image Processed", cv_ptr->image);
//   // //Add some delay in miliseconds. The function only works if there is at least one HighGUI window created and the window is active. If there are several HighGUI windows, any of them can be active.
//   // cv::waitKey(3);
//   // //Publish new image 
//   // //pub.publish(cv_ptr->toImageMsg());
//   // std::cout << "Red Density:" << Density_Red << std::endl;
// }

void CWallFollower::updatecommandVelocity(double linear, double angular)
{
  geometry_msgs::Twist cmd_vel;

  cmd_vel.linear.x  = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_.publish(cmd_vel);
}


bool CWallFollower::controlLoop()
{
  // static int counter = 0;
  // if (counter == 0)
  // {
  //   turnOdom(1.35);
  // }
  // counter++;

  // ------ Static classes ------ 
  static CDriveForward driveFoward(kp_, ki_, kd_);
  static CConcaveCorner concaveSolver;
  static convexState convexState;
  // ------ Next state logic ------ 
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
  linearV = 0.0;
  angularV = 0.0;
  return true;
}

// Function to apply a moving average filter to Lidar data
void CWallFollower::applyMovingAverageFilter() {
  // used to count over the 0->filterWindowSize counts
  static int counter= 0;

  // use to change moving filter for first time through the counts
  static bool first_time = true;

  // check if the moving average needs to be reset
  if(previousState != States::DRIVE_FOWARD || movingAverage[0].empty())
  {
    for(int i=0; i < mFilterWindowSize; i++)
    {
      for (const auto &pair : scanDataMapped)
      {
        movingAverage[i][pair.first] = 0.0;
      }
    }
    first_time = true;
    counter = 0;
  }
  else 
  {
    // Reset the counter after each window size
    // and set first_time to false after counter == factored window
    counter +=1;
    if (counter == mFilterWindowSize)
    {
      counter = 0;
      first_time = false;
    }
  }
  // Fill in moving average data 
  for (const auto &pair : scanDataMapped)
  {
    movingAverage[counter][pair.first] = pair.second;
  }

  // reset filtered data to zeros
  for (const auto &pair : scanDataMapped)
    {
      filteredScanData[pair.first] = 0.0;
    }

  if (first_time)
  {
    for(int i=0; i < counter + 1; i++)
    {

      for (const auto &pair : scanDataMapped)
      {
        filteredScanData[pair.first] += (movingAverage[i][pair.first])/(counter+1);
        //std::cout << filteredScanData[pair.first] << std::endl;
      }
    }   
  }
  else 
  {
    for(int i=0; i < mFilterWindowSize; i++)
    {
      for (const auto &pair : scanDataMapped)
      {
        filteredScanData[pair.first] += movingAverage[i][pair.first]/(mFilterWindowSize);
      }
    }   
  }
}
  
void CWallFollower::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
  // Calculate the rotation angle in yaw using euler angles
  double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
	double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
  double yaw = std::atan2(siny, cosy);
  
  // Change yaw to 0 to 360 from 0-180 -> -180-0
  if (yaw < 0)
    yaw += 2*M_PI;

  mRotationAngle = yaw;
}

bool CWallFollower::turnOdom(double angle)
{
  // we will be sending commands of type "twist"
  geometry_msgs::Twist base_cmd;

  // the command will be to turn at 0.75 rad/s
  base_cmd.linear.x = base_cmd.linear.y = 0.0;
  base_cmd.angular.z = ANGULAR_VELOCITY;
  
  // Angle diference
  double angledif = mRotationAngle - angle;

  // Boolean used to determine if the angle is within the tolerance
  bool done = false;

  // change input angle to be relative to current rotation angle
  if(angledif < 0)
  {
    angle = 2*M_PI+(mRotationAngle-angle);
  }
  else 
  {
    angle = mRotationAngle-angle;
  }
  angledif = mRotationAngle - angle;
  
  // 50 Hz ros rate
  ros::Rate rate(50.0);

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
      // get callbacks
      ros::spinOnce();
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
  ROS_INFO("INIT FIN");

  ros::Rate loop_rate(30); // 30 Hz

  // ros::ok handles ctrl+C interupts 
  // Loop until sensor data is not zero!
  while (ros::ok() && (wallFollower.scanDataMapped[0] < 0.05 && wallFollower.mRotationAngle < 0.05))
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  ROS_INFO("Main Whileloop Start");
  
  while (ros::ok())
  {
    wallFollower.controlLoop();
    // call callbacks
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

//--------------------