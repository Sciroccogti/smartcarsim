/*
 *	VERSION:	SCS-1.0.1
 *	DATE:		2013.Sep.1
 *	AUTHOR:		Yu Kunlin
 */

#ifndef _TRACK_H_
#define _TRACK_H_

#include <ode/common.h>
#include "vector.h"

#define TRACK_WIDTH_OUT		0.45
#define TRACK_WIDTH_IN		(TRACK_WIDTH_OUT-2*LINE_WIDTH)
#define TRACK_STEP		0.05
#define TRACK_HEIGHT		0.01

#define LINE_WIDTH		0.025
#define LINE_LENGTH		0.10

#define BARRIER_HEIGHT		0.005
#define ELEVATION_ANGLE		15
#define ELEVATION_ANGLE_B	12
#define TRANSITION_LENGTH	0.10

#define MAXPOINT		10000
#define MAXSEGMENT		1000

extern sVector Path[MAXPOINT];
extern int PointNum;
extern double TotalLength;
extern double EndLineDistance;
extern sVector Middle[MAXPOINT];
extern double PathSecurity;
extern sVector Path[MAXPOINT];
extern int PathNum;
extern sVector ELineL,ELineR;
extern int ConnectFlag;
extern int MiddleLineFlag;
extern int TrackReverseFlag;

void MakeTrack(dSpaceID space, const char * fname);
void DestroyTrack ();
void DrawTrack();
void DrawPath ();

#endif
