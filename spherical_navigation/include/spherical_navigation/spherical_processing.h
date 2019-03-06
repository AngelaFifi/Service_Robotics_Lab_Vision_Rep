#ifndef SPHERICAL_PROCESSING_H
#define SPHERICAL_PROCESSING_H
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32.h>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <vector>
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/core/core.hpp>
#include <XmlRpc.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <ros/console.h>

namespace enc = sensor_msgs::image_encodings;


using namespace std;
using namespace cv;



class spherical_processing;
class spherical_processing
{
    ros::NodeHandle nh;

    image_transport::ImageTransport it;
    image_transport::Publisher pubi;
    image_transport::Subscriber sub_si;


public:

    /** General Variables*/

    //Camera Matrix

    void image_processing();
    void imageCallback(const sensor_msgs::ImageConstPtr& original_image);


};

#endif //
