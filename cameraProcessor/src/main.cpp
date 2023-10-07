#include "../include/cameraTester/camera.h"

static const char WINDOW[] = "Image Processed";

CCamera::CCamera(std::string image_topic)
 : it_(nh_)
{
  // Init turtlebot3 node
  ROS_INFO("CAMERA INIT");

  //Subscribe to cammer image
  sub_img_ = it_.subscribe( image_topic, QUEUE_SIZE, &CCamera::imageCallback, this);
  
  //Publish the red detection booleen if true or false
  pub_img_ = nh_.advertise<std_msgs::Bool>("Red_bool", 1000);
  
}

CCamera::~CCamera()
{
    //closing proceedure
  cv::destroyWindow(WINDOW);
  ros::shutdown();
}

void CCamera::init()
{
    //Begin WIndow to show image
    cv::namedWindow(WINDOW, cv::WINDOW_AUTOSIZE);

    ROS_INFO("INIT FIN");
}

void CCamera::imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
    //Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    cv_bridge::CvImagePtr cv_ptr;
    
    cv_ptr = cv_bridge::toCvCopy(original_image, sensor_msgs::image_encodings::BGR8);

    //Check image
    int cam_width = cv_ptr->image.cols;
    int cam_height = cv_ptr->image.rows;

    
  //Analysise raw image data initalisation
  double red_pixels = 0;  //number of red pixel in threshold
  double RGB_alpha = 0; //RGB alpha (total of RBG)
  double Density_Red_pixel = 0; //Density of red in pixel

  //Go through all pixels
  for(int t = 0; t < cam_width*cam_height; t++ )
  { 
      //go through all channels (b,g,r) and determine if in threshold to be a red pixel
      int blue = cv_ptr->image.data[0 + 3*t]  ;
      int green = cv_ptr->image.data[1 + 3*t] ;
      int red = cv_ptr->image.data[2 + 3*t] ;
          
      RGB_alpha = blue+green+red;
      Density_Red_pixel = red / RGB_alpha;
        
      if(Density_Red_pixel >= Thresh_Density_Red_pixel){
        red_pixels++;
      }

      //std::cout << "count: " << red_pixels << " || count: " <<pixel_count<<" out: "<<Density_Red<< " red: "<<Thresh_Density_Red<<std::endl;
  }

  //Determine RED Desnity in image

    if(red_pixels >= (cam_height*cam_width)*Thresh_Density_Red_image){
        std::cout << "Red was Detected" << std::endl;
        Detect.data = true;
    }else
    {
        std::cout << "Red was not Detected" << std::endl;
        Detect.data = false;
    }

  //Display the image using OpenCV
  cv::imshow("Image", cv_ptr->image);
  //Add some delay in miliseconds. The function only works if there is at least one HighGUI window created and the window is active. If there are several HighGUI windows, any of them can be active.
  cv::waitKey(3);
  //Publish data 
  pub_img_.publish(Detect);

}

// ------------------------ Main -----------------------------------------------
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "cameraTester");
    static const char image_topic[] = IMAGE_TOPIC;
    //Begin Camera Object
    CCamera camera(image_topic);

    ROS_INFO("Main Started");

    ros::Time::init();

    //Main Access Point for communications
    ros::Rate loop_rate(30); // 30 Hz
    while (ros::ok())
  {
    // callbacks
    ros::spinOnce();
    //ROS_INFO("SCANNING:");
    loop_rate.sleep();
  }

    return 0;
}

//--------------------