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
  mFowardDistanceLimit  = 0.55;   // in m
  mLeftDistanceLimit    = 0.4;   // in m
  mTolerance            = 0.15;   // pm 0.1m
  mAngleTolerance       = 0.01;   // pm 0.05 rad

  ROS_INFO("INIT FIN");
  return true;
}

void CWallFollower::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  // interate over mScanDataRanges
  for (int num = 0; num < 2; num++)
  {
    // If the scanned distance is inf set to range_max
    if (std::isinf(msg->ranges.at(_mScanAngle[num])))
    {
      mScanDataRanges[num] = msg->range_max;
    }
    else
    {
      mScanDataRanges[num] = msg->ranges.at(_mScanAngle[num]);
    }
  }
}

void CWallFollower::updatecommandVelocity(double linear, double angular)
{
  geometry_msgs::Twist cmd_vel;

  cmd_vel.linear.x  = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_.publish(cmd_vel);
}

void CWallFollower::turnLeft()
{
  double snappedRotation;
  if (mRotationAngle < M_PI/4 || mRotationAngle > 3*M_PI/2 + M_PI/4)
  {
    // 315 to 45 deg
    snappedRotation = 0.0;
  }
  else if (mRotationAngle > M_PI/4 && mRotationAngle < M_PI/2 + M_PI/4)
  {
    // 45 to 135
    snappedRotation = M_PI/2;
  }
  else if (mRotationAngle > M_PI/2 + M_PI/4 && mRotationAngle < M_PI + M_PI/4)
  {
    // 135 to 225
    snappedRotation = M_PI;
  }
  else
  {
    // 225 to 315
    snappedRotation = 3*M_PI/2;
  }


  // 90 deg CW turn to the left 
  if (fabs(snappedRotation - 3*M_PI/2) < 1e-2)
  {
    // acute angle case
    turnOdom(2*M_PI-0.0001);
  }
  else 
  {
    // obtuse angle case
    turnOdom(snappedRotation + M_PI/2);
  }
}

void CWallFollower::snapOrientation()
{
  double snappedRotation;
  if ((mRotationAngle < M_PI/4) || (mRotationAngle > 3*M_PI/2 + M_PI/4))
  {
    // 315 to 45 deg
    snappedRotation = 2*M_PI;
  }
  else if (mRotationAngle > M_PI/4 && mRotationAngle < M_PI/2 + M_PI/4)
  {
    // 45 to 135
    snappedRotation = M_PI/2;
  }
  else if (mRotationAngle > M_PI/2 + M_PI/4 && mRotationAngle < M_PI + M_PI/4)
  {
    // 135 to 225
    snappedRotation = M_PI;
  }
  else
  {
    // 225 to 315
    snappedRotation = 3*M_PI/2;
  }

  turnOdom(snappedRotation);
}

void CWallFollower::turnRight()
{
  double snappedRotation;
  if ((mRotationAngle < M_PI/4) || (mRotationAngle > 3*M_PI/2 + M_PI/4))
  {
    // 315 to 45 deg
    snappedRotation = 2*M_PI;
  }
  else if (mRotationAngle > M_PI/4 && mRotationAngle < M_PI/2 + M_PI/4)
  {
    // 45 to 135
    snappedRotation = M_PI/2;
  }
  else if (mRotationAngle > M_PI/2 + M_PI/4 && mRotationAngle < M_PI + M_PI/4)
  {
    // 135 to 225
    snappedRotation = M_PI;
  }
  else
  {
    // 225 to 315
    snappedRotation = 3*M_PI/2;
  }


  // 90 deg CW turn to the right 
  if (fabs(snappedRotation - M_PI/2) < 1e-2)
  {
    // acute angle case
    turnOdom(2*M_PI-0.0001);
  }
  else 
  {
    // obtuse angle case
    turnOdom(snappedRotation - M_PI/2);
  }
}

bool CWallFollower::controlLoop()
{
  /// Implement state machine here!
  // local static variable is set only the first time the function is called.
  static mState currentState = DRIVE_FOWARD;
  static mState nextState;

  // for convex corners
  static int counter = 0;
  static bool counter2 = false;
  static bool counter3 = false;
  switch(currentState)
  {
    case DRIVE_FOWARD:
      if (mScanDataRanges[LEFT] < (mLeftDistanceLimit + mTolerance/2)
          && mScanDataRanges[LEFT] > (mLeftDistanceLimit - mTolerance/2))
      {
        // Within tolerance drive foward
        #ifdef DEBUG
          ROS_INFO("In tolerance :)");
        #endif
        counter2 = false;
        counter3 = false;
        if(mScanDataRanges[CENTER] > mFowardDistanceLimit - mTolerance/2)
        {
          if(mScanDataRanges[LEFT] > mLeftDistanceLimit)
          {
            // Futher away from wall
            updatecommandVelocity(LINEAR_VELOCITY, ANGULAR_VELOCITY/11);
          }
          else 
          {
            // closer to wall
            updatecommandVelocity(LINEAR_VELOCITY, -ANGULAR_VELOCITY/6);
          }
          nextState = DRIVE_FOWARD;
        }
        else
        {
          updatecommandVelocity(0.0, 0.0);
          nextState = RIGHT_TURN;
        }
      }
      else
      {
        #ifdef DEBUG
          ROS_INFO("Not in tolerance");
          ROS_INFO("L: [%f]", (mScanDataRanges[LEFT]));
        #endif

        if (mScanDataRanges[LEFT] < (mLeftDistanceLimit))
         {
          turnLeft();
          nextState = REVERSE;
         }
        else if (mScanDataRanges[LEFT] > mLeftDistanceLimit + 2*mTolerance)
        {
          counter3 = false;
          // encountered convex corner 
          if(!counter2)
          {
            if (counter < 170)
            {
              updatecommandVelocity(LINEAR_VELOCITY, 0.0);
              counter++;
            }
            else 
            {
              updatecommandVelocity(0.0, 0.0);
              counter = 0;
              counter2 = true;
              turnLeft();
            }
          }
          else 
          {
            updatecommandVelocity(LINEAR_VELOCITY, 0.0);
          }
          nextState = DRIVE_FOWARD;
        }
        else
        {
          counter2 = false;
          if(!counter3)
          {
            snapOrientation();
            counter3 = true;
          }
          else 
          {
            /// Find a wall to the left!
            updatecommandVelocity(0.0, 0.0);
            // Turn 90 deg to the left
            turnLeft();
            nextState = FIND_LHS_WALL;
          }
        }
      }
      break;
    case FIND_LHS_WALL:
      // Within tolerance drive foward
      //ROS_INFO("Distance= %f", mScanDataRanges[CENTER]);
      if(mScanDataRanges[CENTER] > mFowardDistanceLimit - mTolerance/2)
      {
        updatecommandVelocity(LINEAR_VELOCITY, 0.0);
        nextState = FIND_LHS_WALL;
      }
      else
      {
        updatecommandVelocity(0.0, 0.0);
        nextState = RIGHT_TURN;
      }
      break;
    case REVERSE:
      if(mScanDataRanges[CENTER] < mFowardDistanceLimit - mTolerance)
      {
        updatecommandVelocity(-LINEAR_VELOCITY, 0.0);
        nextState = REVERSE;
      }
      else
      {
        ROS_INFO("EXITING REVERSE");
        updatecommandVelocity(0.0, 0.0);
        turnRight();
        nextState = DRIVE_FOWARD;
      }
      break;
    case LEFT_TURN:
      // Turn 90 deg to the left
      updatecommandVelocity(0.0, 0.0);
      turnLeft();
      nextState = DRIVE_FOWARD;
      break;
    case RIGHT_TURN:
      // Turn 90 deg to the right
      updatecommandVelocity(0.0, 0.0);
      turnRight();
      nextState = DRIVE_FOWARD;
    default:
      ROS_INFO("Somthing Broke!");
      updatecommandVelocity(0.0, 0.0);
      nextState = DRIVE_FOWARD;
      break;
  }
  currentState = nextState;
  
  //ROS_INFO("State [%d]", currentState);
  #ifdef DEBUG
    ROS_INFO("State [%d]", currentState);
  #endif

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

  ros::Rate loop_rate(50); 

  // ros::ok handles ctrl+C interupts 
  ROS_INFO("Main Whileloop Start");
  
  // Loop until angle is not zero!
  while (ros::ok() && fabs(wallFollower.mRotationAngle) < 1e-2)
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  // orient the robot up
  wallFollower.turnOdom(M_PI);

  while (ros::ok())
  {
    wallFollower.controlLoop();
    // callbacks
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}