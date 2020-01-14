/*
 *	VERSION:	SCS-1.0.1
 *	DATE:		2013.Sep.1
 *	AUTHOR:		Yu Kunlin
 */

#ifndef _CAR_H_
#define _CAR_H_

#include <ode/common.h>
#include "track.h"
#include "vector.h"

#define A_WHEEL_RADIUS		0.025	// wheel radius
#define A_WHEEL_FRONT_WIDTH	0.025	// wheel width
#define A_WHEEL_BACK_WIDTH	0.025	// wheel width

#define B_WHEEL_RADIUS		0.03	// wheel radius
#define B_WHEEL_FRONT_WIDTH	0.025	// wheel width
#define B_WHEEL_BACK_WIDTH	0.04	// wheel width

#define INDUCTANCE_NUM		128	// max number of inductance

#define STARTZ	(TRACK_HEIGHT + 0.05)	// starting height of chassis
#define FMAX	1.0	// car engine fmax

typedef struct sObject {
	dBodyID body;
	dGeomID geom;
} * sObjectID;

extern sObjectID Chassis;
extern sObjectID Wheel_FL, Wheel_FR, Wheel_BL, Wheel_BR;
extern dJointID Joint_FL, Joint_FR, Joint_BL, Joint_BR;
extern int CarReverseFlag;
extern int CarDirection;
extern sVector BatteryPos;
extern sVector BatteryPosR;
extern sVector BatteryPosB;
extern double WheelRadius;
extern double CarWidth;
extern sVector InductancePos [INDUCTANCE_NUM];
extern sVector Magnetic [INDUCTANCE_NUM];
extern int Ip;

void MakeCar (double x, double y, dWorldID world, dSpaceID space);
void ResetCar ();
void MakeBalanceCar (double x, double y, dWorldID world, dSpaceID space);
void DrawCar ();
void DrawBalanceCar ();
void SetCar(double speed, double turn);
void SetBalanceCar(double speedL, double speedR);
sVector CarX ();
sVector CarY ();
sVector CarZ ();
sVector ToCarCoo (sVector v);
sVector ToWorldCoo (sVector v);

#endif
