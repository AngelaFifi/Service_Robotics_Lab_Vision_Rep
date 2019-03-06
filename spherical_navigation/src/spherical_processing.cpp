#include "spherical_processing.h"

Img_Proc::Img_Proc(Point2d istart, Size iROI_sz)
    :ROI_sz(iROI_sz), start(istart), iit(nhi){
    pub_left=iit.advertise("/camera/image_left",1);
    pub_right=iit.advertise("/camera/image_right",1);
}

IplImage * Img_Proc::split_image(cv::Mat input_im, int n)
{

  cv::Mat clone_image, Roi_Img;
  sensor_msgs::ImagePtr msg;
  clone_image=input_im.clone();
  //imshow("Original Image",input_im);
  if (n%2 ==0){
    cv::Rect left_roi(this->start.x,this->start.y,this->ROI_sz.width,this->ROI_sz.height);
    Roi_Img= clone_image(left_roi);
    imshow("Left Image",Roi_Img);
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", Roi_Img).toImageMsg();
    pub_left.publish(msg);
  }
  else {
    cv::Rect right_roi(this->start.x,this->start.y,this->ROI_sz.width,this->ROI_sz.height);
    Roi_Img= clone_image(right_roi);
    imshow("Right Image",Roi_Img);
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", Roi_Img).toImageMsg();
    pub_right.publish(msg);
  }
  IplImage src= Roi_Img.clone();
  IplImage *src_g=cvCloneImage(&src);
  return src_g;

}



bool spherical_processing::get_ocam_model(struct ocam_model *myocam_model, const char *filename)
{
 double *pol        = myocam_model->pol;
 double *invpol     = myocam_model->invpol;
 double *xc         = &(myocam_model->xc);
 double *yc         = &(myocam_model->yc);
 double *c          = &(myocam_model->c);
 double *d          = &(myocam_model->d);
 double *e          = &(myocam_model->e);
 int    *width      = &(myocam_model->width);
 int    *height     = &(myocam_model->height);
 int *length_pol    = &(myocam_model->length_pol);
 int *length_invpol = &(myocam_model->length_invpol);
 FILE *f;
 char buf[CMV_MAX_BUF];
 int i;

 //Open file
 if(!(f=fopen(filename,"r")))
 {
   printf("File %s cannot be opened\n", filename);
   return false;
 }

 //Read polynomial coefficients
 fgets(buf,CMV_MAX_BUF,f);
 fscanf(f,"\n");
 fscanf(f,"%d", length_pol);
 for (i = 0; i < *length_pol; i++)
 {
     fscanf(f," %lf",&pol[i]);
 }

 //Read inverse polynomial coefficients
 fscanf(f,"\n");
 fgets(buf,CMV_MAX_BUF,f);
 fscanf(f,"\n");
 fscanf(f,"%d", length_invpol);
 for (i = 0; i < *length_invpol; i++)
 {
     fscanf(f," %lf",&invpol[i]);
 }

 //Read center coordinates
 fscanf(f,"\n");
 fgets(buf,CMV_MAX_BUF,f);
 fscanf(f,"\n");
 fscanf(f,"%lf %lf\n", xc, yc);

 //Read affine coefficients
 fgets(buf,CMV_MAX_BUF,f);
 fscanf(f,"\n");
 fscanf(f,"%lf %lf %lf\n", c,d,e);

 //Read image size
 fgets(buf,CMV_MAX_BUF,f);
 fscanf(f,"\n");
 fscanf(f,"%d %d", height, width);

 fclose(f);
 return true;
}

void spherical_processing::cam2world(double point3D[3], double point2D[2], struct ocam_model *myocam_model)
{
 double *pol    = myocam_model->pol;
 double xc      = (myocam_model->xc);
 double yc      = (myocam_model->yc);
 double c       = (myocam_model->c);
 double d       = (myocam_model->d);
 double e       = (myocam_model->e);
 int length_pol = (myocam_model->length_pol);
 double invdet  = 1/(c-d*e); // 1/det(A), where A = [c,d;e,1] as in the Matlab file

 double xp = invdet*(    (point2D[0] - xc) - d*(point2D[1] - yc) );
 double yp = invdet*( -e*(point2D[0] - xc) + c*(point2D[1] - yc) );

 double r   = sqrt(  xp*xp + yp*yp ); //distance [pixels] of  the point from the image center
 double zp  = pol[0];
 double r_i = 1;
 int i;

 for (i = 1; i < length_pol; i++)
 {
   r_i *= r;
   zp  += r_i*pol[i];
 }

 //normalize to unit norm
 double invnorm = 1/sqrt( xp*xp + yp*yp + zp*zp );

 point3D[0] = invnorm*xp;
 point3D[1] = invnorm*yp;
 point3D[2] = invnorm*zp;
}

void spherical_processing::world2cam(double point2D[2], double point3D[3], struct ocam_model *myocam_model)
{
 double *invpol     = myocam_model->invpol;
 double xc          = (myocam_model->xc);
 double yc          = (myocam_model->yc);
 double c           = (myocam_model->c);
 double d           = (myocam_model->d);
 double e           = (myocam_model->e);
 int    width       = (myocam_model->width);
 int    height      = (myocam_model->height);
 int length_invpol  = (myocam_model->length_invpol);
 double norm        = sqrt(point3D[0]*point3D[0] + point3D[1]*point3D[1]);
 double theta       = atan(point3D[2]/norm);
 double t, t_i;
 double rho, x, y;
 double invnorm;
 int i;

  if (norm != 0)
  {
    invnorm = 1/norm;
    t  = theta;
    rho = invpol[0];
    t_i = 1;

    for (i = 1; i < length_invpol; i++)
    {
      t_i *= t;
      rho += t_i*invpol[i];
    }

    x = point3D[0]*invnorm*rho;
    y = point3D[1]*invnorm*rho;

    point2D[0] = x*c + y*d + xc;
    point2D[1] = x*e + y   + yc;
  }
  else
  {
    point2D[0] = xc;
    point2D[1] = yc;
  }
}
//------------------------------------------------------------------------------
void spherical_processing::create_perspecive_undistortion_LUT( CvMat *mapx, CvMat *mapy, struct ocam_model *ocam_model, float sf)
{
     int i, j;
     int width = mapx->cols; //New width
     int height = mapx->rows;//New height
     float *data_mapx = mapx->data.fl;
     float *data_mapy = mapy->data.fl;
     float Nxc = height/2.0;
     float Nyc = width/2.0;
     float Nz  = -width/sf;
     double M[3];
     double m[2];

     for (i=0; i<height; i++)
         for (j=0; j<width; j++)
         {
             M[0] = (i - Nxc);
             M[1] = (j - Nyc);
             M[2] = Nz;
             world2cam(m, M, ocam_model);
             *( data_mapx + i*width+j ) = (float) m[1];
             *( data_mapy + i*width+j ) = (float) m[0];
         }
}

//------------------------------------------------------------------------------
void spherical_processing::create_panoramic_undistortion_LUT ( CvMat *mapx, CvMat *mapy, float Rmin, float Rmax, float xc, float yc )
{
     int i, j;
     float theta;
     int width = mapx->width;
     int height = mapx->height;
     float *data_mapx = mapx->data.fl;
     float *data_mapy = mapy->data.fl;
     float rho;

     for (i=0; i<height; i++)
         for (j=0; j<width; j++)
         {
             theta = -((float)j)/width*2*M_PI; // Note, if you would like to flip the image, just inverte the sign of theta
             rho   = Rmax - (Rmax-Rmin)/height*i;
             *( data_mapx + i*width+j ) = yc + rho*sin(theta); //in OpenCV "x" is the
             *( data_mapy + i*width+j ) = xc + rho*cos(theta);
         }
}


spherical_processing::spherical_processing()
    : it(nh)
{
    sub_si = it.subscribe("/richon_theta/image_raw", 1, &spherical_processing::imageCallback,this);



}
spherical_processing::~spherical_processing()
{

  //  destroyWindow("Original Image");
    destroyWindow("Left Image");
    destroyWindow("Right Image");
    cvDestroyAllWindows();
  //  destroyWindow("Undistorted Perspective Image Left");
    //destroyWindow("Undistorted Perspective Image Right");
}

void spherical_processing::imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
    try
    {
        cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }

    image_processing(cv_ptr->image);

}

void spherical_processing::image_processing(cv::Mat input_img)
{
  IplImage* half;
  for(int i=0; i< features.size(); i++){
    half=features.at(i).split_image(input_img,i);

  IplImage*  dst_persp   = cvCreateImage(cvGetSize(half), 8, 3 );   // undistorted perspective and panoramic image
    CvSize size_pan_image = cvSize(1200,400);        // size of the undistorted panoramic image

    CvMat* mapx_persp = cvCreateMat(half->height, half->width, CV_32FC1);
    CvMat* mapy_persp = cvCreateMat(half->height, half->width, CV_32FC1);

    float sf = 4;
    create_perspecive_undistortion_LUT( mapx_persp, mapy_persp, &left_cm, sf );

    cvRemap( half, dst_persp, mapx_persp, mapy_persp, CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS, cvScalarAll(0) );

      if (i%2 ==0)
      {
        cvShowImage( "Undistorted Perspective Image Left", dst_persp );
        cv::Mat m = cv::cvarrToMat(dst_persp);

      }
    else
    cvShowImage( "Undistorted Perspective Image Right", dst_persp );
    cvReleaseImage(&dst_persp);
    cvReleaseImage(&half);
}




    waitKey(0);
}


void spherical_processing::initialize()
{
  Size sz(960,960);
  Point2d cl;
  cl.x=0;
  cl.y=0;
  features.push_back(Img_Proc(cl,sz));
  Point2d cr;
  cr.x=960;
  cr.y=0;
  features.push_back(Img_Proc(cr,sz));

  get_ocam_model(&left_cm,"/home/angela/catkin_ws/src/spherical_navigation/include/calib_results_Left.txt" );
  get_ocam_model(&right_cm, "/home/angela/catkin_ws/src/spherical_navigation/include/calib_results_Right.txt");
//  int i;
//    printf("pol =\n");    for (i=0; i<left_cm.length_pol; i++){    printf("\t%e\n",left_cm.pol[i]); };    printf("\n");
//    printf("invpol =\n"); for (i=0; i<right_cm.length_invpol; i++){ printf("\t%e\n",right_cm.invpol[i]); }; printf("\n");
//    printf("\nxc = %f\nyc = %f\n\nwidth = %d\nheight = %d\n",left_cm.xc,left_cm.yc,left_cm.width,left_cm.height);

}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "spherical_processing");

        spherical_processing object;
        object.initialize();
        ros::spin();

}
