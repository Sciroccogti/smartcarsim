/*
 *	VERSION:	SCS-1.0.1
 *	DATE:		2013.Sep.1
 *	AUTHOR:		Yu Kunlin
 */

#ifndef _UI_H_
#define _UI_H_

#include "vector.h"

#define GLUT_WHEEL_UP	3
#define GLUT_WHEEL_DOWN	4

typedef struct sRoute {
	sVector pos;
	sRoute * next;
	double speed;
} * sRouteID;

extern sRouteID head;
extern sVector ViewPointDelta;

void Play ();
void Key (unsigned char cmd, int x, int y);
void SpecialKeyPress (int key, int x, int y);
void SpecialKeyUp(int key, int x, int y);
void Mouse (int button, int state, int x, int y);
void motion (int x, int y);
void ClearRoute ();
void Free (sRouteID head);
void Reshape (int width, int height);

#endif
