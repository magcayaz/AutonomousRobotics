#include <stdio.h>
#include <string.h>
#include <stdlib.h> 
#include "flockbot_api.h"
#include <math.h>


void findNearWall();
void writeToLCD(char *msg);
int irDist(int sNum);
void wallFacing();


//Globals
int irLF = 0;
int irL = 1;
int irF = 2;
int irR = 3;
int irRF = 4;

//robot values
int turnSpeed = 50;
int driveSpeed = 50;
char LCDmsg[15];

int main()
{
	initialize_api();
	robot_connect("127.0.0.1");
	
	//code here
	
	writeToLCD("Looking nearest surfaceBox");
	//findNearWall();//find and face near wall
	
	int running = 1;
	while (running)
	{
		if(get_high_bump() == 1) 
    {
    	running = 0;
    }
		wallFacing();//keep facing the wall

	}
	
	claw_close();//claw claw
	claw_open();
	claw_close();
	claw_open();
	robot_stop();
	
	//end code
	
	shutdown_api();
	return 0;
}

void writeToLCD(char *msg)
{
	lcd_clear();
	lcd_set_cursor(1,1);
	lcd_write(msg);
}

int irDist(int sNum)
{
	int irSamples = 10;//was 20
	int irValue[irSamples];
	int outofRangeCount = 0;
	double average = 0; 
	double stdDev = 0;
	double retValue = 0;
	
	int iterator;

	printf("irDistance function\n");
	
	for(iterator = 0; iterator < irSamples; iterator++)
	{
		
		irValue[iterator] = get_ir(sNum);
		usleep(30000);
		//usleep(30000);
		// waits for the sensor packet to update (every 35ms at least)
		
		if(irValue[iterator] == -1)
		{
			outofRangeCount++;// count invalid values
			
			//if at least a third are invalid, return invalid
			if (outofRangeCount >= irSamples/3)
			{
				printf("Invalid Values\n");
				return -1;
			}
		}
		else//sum up the values
		{
			average += irValue[iterator];
		}
	}
	
	//calculate average from sum
	average = average / (irSamples - outofRangeCount);
	
	//http://education.mrsec.wisc.edu/research/topic_guides/outlier_handout.pdf
	//calculate stdDev
	for(iterator = 0; iterator < irSamples; iterator++)
	{
		stdDev += (average - (double)irValue[iterator])*(average - (double)irValue[iterator]);
	}
	
	stdDev = (stdDev / (irSamples - outofRangeCount-1));
	stdDev = sqrt(stdDev);
	
	//throw away values > stdDev away, and return average
	for(iterator = 0; iterator < irSamples; iterator++)
	{
		if(irValue[iterator] != -1)//only valid values
		{
			if(abs(average-irValue[iterator])<stdDev)
			{//if within 1 stdDev, use for average
				retValue += irValue[iterator];
			}
			else
			{
				outofRangeCount++;
			}
		}	
	}
	retValue = retValue / (irSamples - outofRangeCount);
	printf("ir %d: %f\n",sNum,retValue);
	return retValue;
}

void findNearWall()
{
	int minDist = 32000;
	int curDist = 32000;
	int minDistAngle = -1;
	int angleFreq = 30;
	int angle;
	for(angle = 0; angle <= 365; angle=angle+angleFreq)
	{
		turn_robot_wait(turnSpeed, angleFreq);  //turn full circle
		
		curDist = irDist(irF);
		printf("angle : %d\n",angle);
		if(curDist < minDist && curDist != -1)
		{
			minDist = curDist;
			minDistAngle = angle;
		}
		snprintf(LCDmsg, sizeof(LCDmsg), "%d", curDist);
		writeToLCD(LCDmsg);
	}
	
	//Out of time, but perhaps bug?
	//if (minDistAngle > 180)//pick which way to turn toward min agnle
	//{
	//	minDistAngle -= 180;
	//	turn_robot_wait(turnSpeed*-1, minDistAngle);
	//}
	//else
	//{
		turn_robot_wait(turnSpeed, minDistAngle);
	//}
	
	printf("a: %d D: %d\n", minDistAngle,minDist);	
}

void wallFacing()
{
	int goalDist = 18;
	int actDist = 0;
	int distError = 0;
	int dirError = 0;
	int lfDist = 0;
	int rfDist = 0;
	
	double Kp = 0.03 * driveSpeed;
	double KpDir = 0.005 * driveSpeed;
	double Kd = 0.05 * driveSpeed;
	
	actDist = irDist(irF);
	lfDist = irDist(irLF);
	rfDist = irDist(irRF);
	
	if(actDist == -1)
	{//if out of range, set large error
		actDist = goalDist + 60;
	}
	if(lfDist == -1)
	{//if out of range, set large error
		lfDist = 80;
	}
	if(rfDist == -1)
	{//if out of range, set large error
		rfDist = 80;
	}

	distError = actDist-goalDist;
	dirError = lfDist - rfDist;
	
	move_wheels(Kp*distError+KpDir*dirError,Kp*distError-KpDir*dirError);
}


