#ifndef SPHERICAL_PROCESSING_H
#define SPHERICAL_PROCESSING_H
#include <string>
#include <exception>
#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32.h>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <fstream>
#include <ctime>
#include "opencv2/core/utility.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/core/core.hpp>
#include <XmlRpc.h>
#include "opencv2/imgcodecs.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <ros/console.h>
#include </usr/include/armadillo>
//#include </Desktop/armadillo-9.200.7/include>

namespace enc = sensor_msgs::image_encodings;

using namespace arma;
using namespace std;
using namespace cv;

#define CMV_MAX_BUF 1024
#define MAX_POL_LENGTH 64

struct ocam_model
{
  double pol[MAX_POL_LENGTH];    // the polynomial coefficients: pol[0] + x"pol[1] + x^2*pol[2] + ... + x^(N-1)*pol[N-1]
  int length_pol;                // length of polynomial
  double invpol[MAX_POL_LENGTH]; // the coefficients of the inverse polynomial
  int length_invpol;             // length of inverse polynomial
  double xc;         // row coordinate of the center
  double yc;         // column coordinate of the center
  double c;          // affine parameter
  double d;          // affine parameter
  double e;          // affine parameter
  int width;         // image width
  int height;        // image height
};

class Img_Proc{

protected:
    Point2d start;
    Size ROI_sz;
    ros::NodeHandle nhi;
    image_transport::ImageTransport iit;

public:
  //  void compute_vanishing_points(cv::Mat m);
    image_transport::Publisher pub_left, pub_right;
    Img_Proc(Point2d start, Size ROI_sz);
    IplImage * split_image(cv::Mat input_im, int n);
 };



class spherical_processing
{

protected:
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber sub_si;
    cv_bridge::CvImagePtr cv_ptr;

public:

    /** General Variables and Methods*/
    cv::Mat temp;
    cv::Mat clone_image;
    std::vector<Img_Proc> features;
    ocam_model left_cm, right_cm;
    void cam2world(double point3D[3], double point2D[2], struct ocam_model *myocam_model);
    bool get_ocam_model(struct ocam_model *myocam_model, const char *filename);
    void world2cam(double point2D[2], double point3D[3], struct ocam_model *myocam_model);
    void create_perspecive_undistortion_LUT( CvMat *mapx, CvMat *mapy, struct ocam_model *ocam_model, float sf);
    void create_panoramic_undistortion_LUT ( CvMat *mapx, CvMat *mapy, float Rmin, float Rmax, float xc, float yc );


    spherical_processing();
    ~spherical_processing();
    void image_processing(cv::Mat input_im);
    void imageCallback(const sensor_msgs::ImageConstPtr& original_image);
    void initialize();

};

#endif //SPHERICAL_PROCESSING_H
