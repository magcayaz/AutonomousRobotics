/*ROS pioneer driver node skeleton code
* by Laura Hovatter
* To run this, type the below commands in separate terminals:
* roscore
* rosrun rosaria RosAira _port:=/dev/ttyUSB0
* rosrun driver_node driver_node
*/
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <time.h>

#define _USE_MATH_DEFINES
using namespace std;
std::string frame_id("gmu"); 

//Global Variables
float newLinX = 0.0;
float newAngZ = 0.0;
clock_t oldtime = clock();
double lasterr;
//Callback methods - This is where we read in data and do appropriate calculations

void lidarCallback(const sensor_msgs::LaserScan scan_msg)
{
 // printf("Got Lidar\n");
}

void sonarCallback(const sensor_msgs::PointCloud pc_msg)
{
	//Distance Variables	
	double frontVelCap = .2;
	double frontVelMin = .01;	
	double frontDist = .75; // follow distance goal
	double frontDistError = 0; 
	double frontDistDError = 0; 	
	double frontDistKp = 0.2;
	double frontDistKd = 0.2;	
	double xVel = 0;
	double timechange = clock()-oldtime;
	oldtime = clock();

	//Direction Variables
	double turnVelCap = 0.2;
	double turnVelMin = 0;

	//double turnAngle =0;//turn goal
	double turnAngleError = 0;
	double turnAngleKp = 0.2;
	double aVel = 0;
	double s2 = 0;
	double s3 = 0;
	double s4 = 0;
	double s5 = 0;
	double sCap = 2.0;
	

	//Distance
	frontDistError = (pc_msg.points[3].x+pc_msg.points[4].x)/2 - frontDist;
	frontDistDError = (frontDistError-lasterr)/timechange;
	lasterr = frontDistError;	
	xVel = frontDistError * frontDistKp + frontDistDError*frontDistKd;

	//Max Cap	
	if (xVel > frontVelCap)
		xVel = frontVelCap;
	else if (xVel < -1 * frontVelCap)
		xVel = -1 * frontVelCap;
	//Min Cap
	else if (abs(xVel) < frontVelMin)
		xVel = 0;		
	
	//Direction	
	//save sonar Values	
	s2 = pc_msg.points[2].x;
	s3 = pc_msg.points[3].x;
	s4 = pc_msg.points[4].x;
	s5 = pc_msg.points[5].x;
		
	//cap sonar values
	if (s2 > sCap)
		s2 = sCap;
	if (s3 > sCap)
		s3 = sCap;
	if (s4 > sCap)
		s4 = sCap;
	if (s5 > sCap)
		s5 = sCap;
	
	turnAngleError = ((s3-s4)+(s2 - s5))/2;	
		
	aVel = turnAngleError * turnAngleKp;
	if (aVel > turnVelCap)
		aVel = turnVelCap;
	else if (aVel < -1*turnVelCap)
		aVel = -1*turnVelCap;
	
	//Defaults
	printf("xVel: %f\n", xVel);
	printf("Xaverage: %f\n", (pc_msg.points[3].x+pc_msg.points[4].x)/2);
	newLinX = xVel;
	newAngZ = -1*aVel;
}

void poseCallback(const nav_msgs::Odometry pose_msg)
{
//printf("Got Pose/n");
}


//The main function of the node
int main(int argc, char **argv)
{

//Initiate the node 
  ros::init(argc, argv, "driver_node");
  ros::NodeHandle n;
  
//Subscribe to topics
 // ros::Subscriber subLD = n.subscribe("scan", 1000, lidarCallback); //LiDar
 ros::Subscriber subSonar = n.subscribe("/RosAria/sonar", 1000, sonarCallback); //Sonar
 ros::Subscriber subPose = n.subscribe("/RosAria/pose", 1000, poseCallback); //Pose
ros::Publisher cmd_vel_publisher = n.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 1);

//Rate of data stream - DO NOT TOUCH
  ros::Rate r(10);
  
geometry_msgs::Twist newVelocity; //Declares the new Twist type object
//Only worry about the Linear X and Angular Z, the rest is set to 0.0
newVelocity.linear.y = 0.0;
newVelocity.linear.z = 0.0;
newVelocity.angular.x = 0.0;
newVelocity.angular.y = 0.0; 

//While the node is running, read in the data and publish velocity
  while (!ros::isShuttingDown())
  {
//set the velocity of the pioneer
newVelocity.linear.x = newLinX;
newVelocity.angular.z = newAngZ;

cmd_vel_publisher.publish(newVelocity); 

//DO NOT TOUCH
    ros::spinOnce();
    r.sleep();
  }
  
  return 0;
  }

