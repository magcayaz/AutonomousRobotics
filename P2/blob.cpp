// this file was adopted form kevins opencv_test code. some of the other code is stolen from opencv documentations 
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <pthread.h>

/* BEGIN - Do Not Touch Block */
#include "flockbot_api.h"  // Flockbot API
#include "flockbot_camera.hpp"  // Critical, adds in the OpenCV needed helpers

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace cv; // Uses the cv namespace for convenience

VideoCapture* capture; // Capture device pointer (camera)
Mat src; // Mat to store the captured image in
/* 
 * Standard OpenCV Setup
 * END - Do not touch block 
 */

int main() 
{
  int index;
  Mat green;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  /// Approximate contours to polygons + get bounding rects and circles
  vector<vector<Point> > contours_poly( contours.size() );
  vector<Rect> boundRect( contours.size() );
  vector<Point2f>center( contours.size() );
  vector<float>radius( contours.size() );
  /* BEGIN - Do Not Touch Block */
  initialize_api(); // Initializes the Robot API
  robot_connect("127.0.0.1"); // Connects to the Daemon, needs to be 127.0.0.1
  initialize_camera(); // Initializes the camera device

  capture = new VideoCapture(0);
  /* END - Do Not Touch Block */
  if(capture == NULL)
  {
    printf("Sorry, did not get a capture device\n");
    exit(1);
  }

  // Camera setup
  set_resolution(800,600,15); // Helper function to set resolution. (width,height,fps)

  // How to take a picture
//  while(1){
//	snapshot(src); // This is the only way to take a picture.  Places RGB image in Mat src
//  	inRange(src, Scalar(0, 11, 0), Scalar(2, 86, 10),green);
//	findContours(green, contours, hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
//        /// Approximate contours to polygons + get bounding rects and circles
// 	for( int i = 0; i < contours.size(); i++ )
//	     { approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
//	       boundRect[i] = boundingRect( Mat(contours_poly[i]) );
//       	       minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
//     	       //for( std::vector<char>::const_iterator i = center.begin(); i != center.end(); ++i)
//    	//	std::cout << *i << ' ';
//	}
//	
//  }
  // Sample OpenCV Command
  //imwrite("test.jpg", src);

  /* BEGIN - Do Not Touch Block */
  shutdown_camera();
  shutdown_api();
  return 0;
  /* END - Do Not Touch Block */
}
