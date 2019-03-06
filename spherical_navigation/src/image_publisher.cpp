#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;

    cv_bridge::CvImage cv_image;
    cv_image.image = cv::imread("/home/angela/Documents/Spherical_CaM/image.jpg",CV_LOAD_IMAGE_COLOR);
    cv_image.encoding = "bgr8";
    sensor_msgs::Image ros_image;
    cv_image.toImageMsg(ros_image);

  ros::Publisher pub = nh.advertise<sensor_msgs::Image>("/richon_theta/image_raw", 1);
  ros::Rate loop_rate(5);

  while (nh.ok())
  {
    pub.publish(ros_image);
    loop_rate.sleep();
  }
}
