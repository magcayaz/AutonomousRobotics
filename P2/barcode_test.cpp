/* 
 * barcode_test.cpp
 *
 *  This program uses the Barcode library to find all of the barcoded cans
 *   in the camera view and print out their locations and distances.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

/* BEGIN - Do not touch block */
#include <pthread.h> // Uses pthreads to run the barcode processing library

#include "flockbot_api.h" // Allows the use of the robot API
#include "barcode.hpp"    // Allows the use of the barcode API

int vision_running = 1;  // This governs the main barcode processing loop
pthread_t vision_thread; // Thread to run the barcode processing module
/* 
 * The above code is necessary for the processing thread.
 * END - Do not touch block 
 */

int image_width = 800;
int image_height = 600;

// Returns distance (x,y) in cm
double * dist_to_can(int can_x, int can_y)
{
  // Calibrated to a tilt of 90
  static double distance[2];
  double zdist = 18.0;
  double m = 802.0;
  double theta = atan((can_y-300)/m);  
  double bearing = atan(-(400.0 - can_x)/m);
  distance[0] = zdist/tan(theta);           //dist x
  distance[1] = -distance[0]*tan(bearing);        //dist y
  //convert to world coordinates.
  return distance;
}

//function to search an array for the given value. 
bool inarray(int array[3], int value)
{
  int i;
  for(i = 0; i<3; i++){
    if(array[i] == value){
      return true;
    }
  }
  return false;
}

int main() 
{
  int index;
/* BEGIN - Do not touch block */
  initialize_api(); // Initializes the robot API
  robot_connect("127.0.0.1"); // This always needs to be 127.0.0.1 to connect to the robot
/* 
 * The above code connects to the API 
 * END - Do not touch block
 */

 /* This following function configures the barcode module for use.  You can 
  *  use the values in there now as good defaults. */
  barcode_configure(2 /*number of digits*/, 10 /*barcodes per pass*/, 2 /*kept passes*/, 
              2 /*skip pixel width*/, 2 /*min bar width*/, 100 /* Allowed skew */, -1 /*Otsu thresholding*/,
                          image_width /*image width*/, image_height /*image height*/);
  sleep(2);
  int degrees = 0;
  int** barcodes; // Pointer for a two-dimensional array, will hold returned barcodes
  int count = 0; // Number of barcodes detected in the snapshot
  int* xy;
  double dist_x; // x distance in cm to barcode from flockbot
  double dist_y; // y distance in cm to barcode from flockbot
  double glob_x; //global x coordinates
  double glob_y; //global y coordinates
  int xPix; // pixel coordinate (x)
  int yPix; // pixel coordinate (y)
  double *d;  //pointer for the dist array
  int oldBarcodes[3] = {0,0,0};       //to remember the barcodes we have seen.
  int barcodesSeen = 0;                //number of barcodes seen
  double posBarcodes[6] = {0.0,0.0,0.0,0.0,0.0,0.0}; //x1, y1, x2, y3, x3, y3 in world coordinates
  double centroid[2] = {0.0,0.0};      //center vector
  double centroidDir = 0.0;             //direction of the centroid 
  double centroidMag = 0.0;           //magnitude of the centroid 
  camera_set(90); // Sets the tilt on the camera to 75, which is slightly downward

  /* BEGIN - Do not touch block */
  pthread_create(&vision_thread, NULL, barcode_start, NULL);
  /* 
   * This starts the barcode module's background processing.  
   * END - Do not touch block
   */

  // Nothing special here, loops 100 times to read barcodes.
  //  Really, this is just a demo program, so it arbitrarily runs 100.

  while(degrees <= 360)
  {
    //if(count%1000 == 0)
    printf("On degrees %d\n", degrees);

//    barcode_frame_wait_start(); // Resets the frame wait counter (waits for next full frame)
    barcode_frame_wait(); // Waits for the next frame from the current counter
    
    barcodes = barcode_get_barcodes(); // Retrieves the barcodes in the 2D array

    int num_codes = (int)barcodes[0];  // Number of barcodes detected
    int num_digits = (int)barcodes[1]; // Number of digits per barcode (will be 2)
    double dist;
    int i;

    /* Runs through the barcodes detected, which start at index [2] */
    for(i = 2; i < num_codes+2; i++)
    {
      //printf("Detected %d barcodes %d digits long:\n", num_codes, num_digits);
      int j;

      /* This block converts the raw digits into actual numbers */
      uint32_t sum = 0;
      //printf("\tValue: ");
      for(j = 0; j < num_digits; j++)
      {
        sum += barcodes[i][j] * pow(10,(num_digits-1)-j);
        //printf("%d", barcodes[i][j]);
      }
      /* At this point, your digit array (example: {3.4}) will now be in sum as an int (ex: 34) */

      //printf(" = %d, loc: (%d,%d)->(%d,%d), direction: %d\n",
      //  sum,
      //  barcodes[i][j+0], barcodes[i][j+1],
      //  barcodes[i][j+2], barcodes[i][j+3],
      //  barcodes[i][j+4]);
    
      /* Prints out the current (x,y) and distance to the current barcode */
      xy = barcode_get_cur_xy(sum);
      //finding the bottom coordinates of the can
      yPix = barcodes[i][j+3];
      xPix = (barcodes[i][j+0]+barcodes[i][j+2])/2;
      //printf(" location: (%d,%d)",xPix, yPix );
      d = dist_to_can(xPix,yPix); //returns the coordinates of the can in a vector.
      dist_x = *d;
      dist_y = *(d+1);
      
      //printf("X distance to can: %.2lf\n", dist_x);
      //printf("Y distance to can: %.2lf\n", dist_y);
      //printf("distance to can: %.2lf\n", sqrt(dist_x*dist_x+dist_y*dist_y));
      

      if(barcodesSeen != 3 && !inarray(oldBarcodes, sum)){  //barcodesSeen == 0 ||, this condition could be added for performance 
        oldBarcodes[barcodesSeen] = sum;
        //rotate distances by degrees to get world coordinates
        // Rotate the x and y displacements from vehicle's coordinates to world coordinates. 
        glob_x = dist_x*cos(((M_PI)/180.0)*degrees) - dist_y*sin(((M_PI)/180.0)*degrees);
        glob_y = dist_x*sin(((M_PI)/180.0)*degrees) + dist_y*cos(((M_PI)/180.0)*degrees);
        posBarcodes[barcodesSeen*2]   = glob_x;
        posBarcodes[barcodesSeen*2+1] = glob_y;
        //after about an hour of trying to fix my rotation problems I decided to use a different approach.
        if(degrees >90 && degrees < 180){
          posBarcodes[barcodesSeen*2]   = -glob_y;
          posBarcodes[barcodesSeen*2+1] = -glob_x;
        }
        if(degrees >180 && degrees < 270){
          posBarcodes[barcodesSeen*2]   = glob_y;
          posBarcodes[barcodesSeen*2+1] = glob_x;
        }
        if(degrees >270 && degrees < 360){
          posBarcodes[barcodesSeen*2]   = -glob_y;
          posBarcodes[barcodesSeen*2+1] = -glob_x;
        }
        printf(" Barcode value and xy pos:%d (%.2lf,%.2lf)\n",oldBarcodes[barcodesSeen],posBarcodes[barcodesSeen*2],posBarcodes[barcodesSeen*2+1] );
        barcodesSeen++;
      }



      //free(d);  // ALWAYS FREE
      free(xy); // ALWAYS FREE
	
      /* Prints out the last (x,y) of the current barcode */
      xy = barcode_get_last_xy(sum);
      printf("\t\tlast x/y loc: (%d,%d)\n", xy[0], xy[1]);
      free(xy); // FREE ME
      
      
      
    }
    degrees += 10;      //adding 10 degrees to current angle
    turn_robot_wait(10, 10); //turning 10 more degrees
    count++; /* This is just the counter on the arbitrary looper, not really important*/
     
    /* BEGIN - Do not touch block */
    free(barcodes);
    /*
     * This frees the barcodes.  Memory leaks will destroy you 
     * END - Do not touch block
     */
  }
  turn_robot_wait(10, 20);  //added this here because for some reason at 360 degrees 
                                //we were not getting back to where we started on (robot 1)
  sleep(2);
  centroid[0] = ((posBarcodes[0]+posBarcodes[2]+posBarcodes[4])/3.0); //x center
  centroid[1] = ((posBarcodes[1]+posBarcodes[3]+posBarcodes[5])/3.0); //y center
  centroidDir = atan(centroid[1]/centroid[0])*(180/(M_PI));            // finding the direction
  centroidMag = sqrt(centroid[0]*centroid[0]+centroid[1]*centroid[1]) ;// finding the magnitude
  turn_robot_wait(30, centroidDir);       //turning towards the centroid
  move_distance_wait(30, centroidMag );   //moving to the centroid
  /* BEGIN - Do not touch block */
  vision_running = 0; // Tells the background thread to nicely finish
  pthread_join(vision_thread, NULL); // Waits nicely for the barcode module to shutdown
  shutdown_api(); // Shuts down the robot API
  return 0; // Returns 0
  /*
   * This frees the barcodes.  Memory leaks will destroy you 
   * END - Do not touch block
   */
}
