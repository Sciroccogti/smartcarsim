/*
 *	LIB VERSION:	SCS-1.0.1
 *	API VERSION:	1.0
 *	DATE:		2013.Sep.1
 *	AUTHOR:		Yu Kunlin
 */

#ifndef _API_H_
#define _API_H_

#define VERSION	"SCS 1.0.1"

#include "draw.h"
#include "simulation.h"
#include "common.h"
#include "vector.h"

extern pFunc AiFunc;
extern pFunc DisplayFunc [CUSTOMWINDOWNUM];
extern pFunc InitFunc;
extern char TrackName[255];

extern sVector CurrentSpeed;
extern sVector LastSpeed;

/*	Initial Func	*/
void sSetCar (CarType ct);	// Set type of car, Which can be "camera" or "electromagnetic" or "balance"
void sSetAiFunc (void f());	// Set call back function of ai
void sSetInitFunc (void f());	// Set initial call back function
void sSetTrack (const char * s);// Set the path of track file
void scsMainLoop (int * argc, char *argv[]);	// Run SCS main loop

/*	Enable function Func	*/
void sEnableRoute ();		// Record route of chassis and draw
void sEnableReverse ();		// Set the running direction of the car
void sEnablePath ();		// Show the optimized path
void sEnablePath (double security);// Set security which means the distance between car and border of track
void sEnableMiddleLine ();	// Set Middle Line

/*	Physics Func	*/
void sSetCamera (sVector pos);	// Set the position of camera
void sSetCCD (sVector pos);
void sSetDepressionAngle (double depressionangle);// Set the depression angle of camera
void sSetBatteryPosition (sVector pos);// Set position of battery

/*	Motor Func	*/
void sSetMotor (int voltage);	// Set average speed of motors
void sSetMotorL (int voltage);	// Set speed of left motor
void sSetMotorR (int voltage);	// Set speed of right motor
void sSetServoDir (int dir);	// Set servo motor's direction to set steer


/*	Sensor Func	*/
double sGetASpeedL ();		// Get angular speed of left motor
double sGetASpeedR ();		// Get angular speed of right motor
double sGetASpeed ();		// Get average angular speed of motors
double sGetSpeedL ();		// Get linear speed of left motor
double sGetSpeedR ();		// Get linear speed of right motor
double sGetSpeed ();		// Get average linear speed of motors
sVector sGetAngularSpeed ();	// Get angular speed of chassis of car
sVector sGetAcc ();		// Get accelerate of car
sVector sGetMagnetic (sVector pos);	// Get the magnetic strength (vector) of the position
int sGetReedSwitch ();	// Get the state of reed switch

/*	Graph Func	*/
void sGetGraph (unsigned char graph[GRAPH_HEIGHT][GRAPH_WIDTH]);	// Get graph, no return value
void sGetLine (unsigned char line[GRAPH_WIDTH]);			// Get the middle line of graph

/*	Custom Window Func	*/
void sEnableCustomWindow (int num);	// Show n customs window to draw something
void sEnableCustomWindow ();	// Show n(n==1) custom window to draw something
void sSetDisplayFunc (void f(), int num);// Set call back function of display to the numth window
void sSetDisplayFunc (void f());// Set call back function of display to the first window

/*	Register Func	*/
void sRegister (const char * s);	// To register this software

#endif
