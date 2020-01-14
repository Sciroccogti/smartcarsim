/*
 *	VERSION:	SCS-1.0.1
 *	DATE:		2013.Sep.1
 *	AUTHOR:		Yu Kunlin
 */

#ifndef _DRAW_H_
#define _DRAW_H_

#include "vector.h"

#define CUSTOMWINDOWNUM 16

#define WINDOW_POS_X		100
#ifdef	WIN32
#	define WINDOW_POS_Y		10
#else
#	define WINDOW_POS_Y		30
#endif

#define WINDOW_WIDTH_GOD_0		(int)(800)
#define WINDOW_HEIGHT_GOD_0		(int)(600)

#define WINDOW_WIDTH_GOD_1		(int)(640)
#define WINDOW_HEIGHT_GOD_1		(int)(480)

#define WINDOW_WIDTH_GOD_2		WINDOW_WIDTH_CUSTOM
#define WINDOW_HEIGHT_GOD_2		WINDOW_HEIGHT_CUSTOM

#define WINDOW_WIDTH_CAR		GRAPH_WIDTH
#define WINDOW_HEIGHT_CAR		GRAPH_HEIGHT

#define WINDOW_WIDTH_CUSTOM		QVGA_WIDTH
#define WINDOW_HEIGHT_CUSTOM	QVGA_HEIGHT

typedef void (*pFunc) ();

enum ViewType {
	over,
	car,
	bird,
	birdf,
	hard,
	back,
	overf,
	soft,
	mouse,
};

enum DriveMode {
	play,ai,debug,
};

enum CarType {
	camera,electromagnetic,balance,
};

extern CarType cartype;
extern int RouteFlag;
extern int CustomWindowFlag;
extern int CustomWindowNum;
extern int PathFlag;
extern int WinCustom[CUSTOMWINDOWNUM];
extern int WinCar;
extern int WinGod;
extern sVector ViewPoint;
extern sVector CameraPos;
extern double DepressionAngle;
extern ViewType viewtype;
extern DriveMode drivemode;
extern double VirtualSpeed;
extern double distance;
extern double pitch;
extern double height;
extern double h;
extern double hpr[3];

void ClearViewVar ();
void DrawInit();

#endif
