# Sperical Navigation ROS Package
This ROS node has been tested in Ubuntu 16.04 and ROS kinetic 

At the moment there are two nodes:

1. image_publisher 
This node reads an image which should be published by running gscam package and publish a topic called "/richon_theta/image_raw"
Note: you can use any other package to publish the image 

2. spherical_processing
This node implements the OCamCalib: Omnidirectional Camera Calibration by Scaramuzza.
All the functions have been implemented, but at the moment the node reads the images containing the two fish eye cameras and publishes the two undistorted images.
Please notice that you should calibrate your camera and change the path of the calibrtations file


