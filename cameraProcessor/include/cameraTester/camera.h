#ifndef CAMERA_H
#define CAMERA_H

#define QUEUE_SIZE 1
#define IMAGE_TOPIC "/camera/rgb/image_raw"
#define THRESH_RED_IMAGE 0.7
#define THRESH_RED_PIXEL 0.7

// Are all these includes nessisary?
#include "ros/ros.h"
#include <iostream>
#include <string>     
#include <array>                         // for mScanAngles
//Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>
//Use cv_bridge to convert between ROS and OpenCV Image formats
#include <cv_bridge/cv_bridge.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
//Include headers for OpenCV Image processing
#include <opencv2/imgproc/imgproc.hpp>
//Include headers for OpenCV GUI handling
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Bool.h>


class CCamera
{
    private:
    ros::NodeHandle nh_;
    ros::Publisher pub_img_;

    image_transport::ImageTransport it_;
    image_transport::Subscriber sub_img_;

    void imageCallback(const sensor_msgs::ImageConstPtr& original_image);

    //Camera Info
    int cam_width; //pixel Width
    int cam_height; //pixel height

    //msgs to detect if red or not
    std_msgs::Bool Detect;

    //Thresholds for red
    double Thresh_Density_Red_pixel = THRESH_RED_PIXEL;
    double Thresh_Density_Red_image = THRESH_RED_IMAGE;

    public:
    //initalise members
    CCamera(std::string image_topic);

    //exit strategy 
    ~CCamera();

    //initalise subscribers and publishers
    void init();

};

#endif // WALLFOLLWER_H 