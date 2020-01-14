/*
 *	VERSION:	SCS-1.0.1
 *	DATE:		2013.Sep.1
 *	AUTHOR:		Yu Kunlin
 */

#include <stdio.h>

#include <ode/ode.h>
#include <GL/freeglut.h>

#include "ui.h"
#include "draw.h"
#include "simulation.h"
#include "car.h"
#include "key.h"
#include "api.h"

	sRouteID head = NULL;
	
	static int prex = 0;
	static int prey = 0;
	static int LeftFlag = 0;
	static int RightFlag = 0;
	static int MiddleFlag = 0;

	sVector ViewPointDelta;

	// state of arrow key
	static int KeyState[4] = {0,0,0,0};

void Play ()	// capture input from arrow key to control the car
{
	if (cartype==balance) {
		MotorDutyL = 0;
		MotorDutyR = 0;
		return;
	}
	static double speed=0.0,AimTurn,turn=0.0;
	if (KeyState[0])	speed += ( 20.0 - speed)/30.0;
	else if (KeyState[1])	speed += (-20.0 - speed)/30.0;
	else {
		if (speed>0) speed -= 0.3;
		if (speed<0) speed += 0.3;
	}

	AimTurn = 0.0;
	if (KeyState[2])	AimTurn -= 70;
	if (KeyState[3])	AimTurn += 70;
	turn += (AimTurn-turn)/10.0;

	if (turn>0) {
		MotorDutyL = (int)speed;
		MotorDutyR = (int)(speed*(1-turn/200.0));
	} else {
		MotorDutyL = (int)(speed*(1+turn/200.0));
		MotorDutyR = (int)speed;
	}
	if (!CarDirection)
			ServoDir = (int)turn;
	else	ServoDir = -(int)turn;
}

void Up ()
{
	const double *pos;
	pos = dBodyGetPosition (Wheel_FL->body);
	if (pos[2]<TRACK_HEIGHT+WheelRadius-0.005)
		dBodyAddForce (Wheel_FL->body,0,0,100.0);
	pos = dBodyGetPosition (Wheel_FR->body);
	if (pos[2]<TRACK_HEIGHT+WheelRadius-0.005)
		dBodyAddForce (Wheel_FR->body,0,0,100.0);
	pos = dBodyGetPosition (Wheel_BL->body);
	if (pos[2]<TRACK_HEIGHT+WheelRadius-0.005)
		dBodyAddForce (Wheel_BL->body,0,0,100.0);
	pos = dBodyGetPosition (Wheel_BR->body);
	if (pos[2]<TRACK_HEIGHT+WheelRadius-0.005)
		dBodyAddForce (Wheel_BR->body,0,0,100.0);
}

void Free (sRouteID head)	// free the rest route
{
	static sRouteID p = NULL, q;
	if (!p && head) {		// if there is no rest route, free head
		p = head->next;
		head->next = NULL;
	}
	for (int i=0; i<50 && p; i++) {	// free a little every time
		q = p->next;
		p->next = NULL;
		free (p);
		p = q;
	}
}

void ClearRoute ()
{
	Free (head);
}

void Key (unsigned char cmd, int x, int y)
{
	static double vs = 0.0;
	if (cmd>='a' && cmd<='z') cmd += 'A' - 'a';
	if (cmd!=ESCAPE)	printf("KEY: %c\n",cmd);
	else			printf("KEY: ESCAPE\n");
	switch (cmd) {
		case '`':case '~':	viewtype = over;	ClearViewVar ();	break;
		case '1':case '!':	viewtype = car;		ClearViewVar ();	break;
		case '2':case '@':	viewtype = bird;	ClearViewVar ();	break;
		case '3':case '#':	viewtype = birdf;	ClearViewVar ();	break;
		case '4':case '$':	viewtype = hard;	ClearViewVar ();	break;
		case '5':case '%':	viewtype = back;	ClearViewVar ();	break;
		case '6':case '^':	viewtype = overf;	ClearViewVar ();	break;
		case '7':case '&':	viewtype = soft;	ClearViewVar ();	break;
		case '0':case ')':	viewtype = mouse;
							h += hpr[0];
							pitch = hpr[1];
							height = ViewPoint.GetZ ();				break;
		case 'A':	if (drivemode!=ai) {
					if (AiFunc) {
						drivemode = ai;
						printf ("AI: Enabled.\n");
					}else	printf ("AI: There is no AI function.\n");
				} else {
					drivemode = play;
					printf ("AI: Disabled.\n");
				}
							break;
		case 'D':	if (drivemode!=debug) {
					if (AiFunc) {
						drivemode = debug;
						printf ("DEBUG: debug mode enabled.\n");
					}else	printf ("AI: There is no AI function.\n");
				}
				else {
					if (AiFunc)	drivemode = ai;
					else		drivemode = play;
					printf ("DEBUG: debug mode disabled.\n");
				}
							break;
		case 'T':	if (TrackReverseFlag)	TrackReverseFlag = 0;
				else			TrackReverseFlag = 1;
		case 'R':	ResetSimulation();	break;
		case 'C':	if (RouteFlag) {
						ClearRoute ();
						printf("ROUTE: route cleared.\n");
						RouteFlag = 0;
					} else {
						printf("ROUTE: Route enabled.\n");
						RouteFlag = 1;
					}
					break;
		case '=':
		case '+':
				if (VirtualSpeed!=0.0)
					if (VirtualSpeed>=1.0)	VirtualSpeed += 1.0;
					else			VirtualSpeed *= 2.0;
				else
					if (vs>=1.0)	vs += 1.0;
					else			vs *= 2.0;
				printf("VirtualSpeed = %3.2f\n",VirtualSpeed);	break;
		case '_':
		case '-':
				if (VirtualSpeed!=0.0)
					if (VirtualSpeed<=1.0)	VirtualSpeed /= 2.0;
					else			VirtualSpeed -= 1.0;
				else
					if (vs<=1.0)	vs /= 2.0;
					else			vs -= 1.0;
				printf("VirtualSpeed = %3.2f\n",VirtualSpeed);	break;
		case 'P':	if (VirtualSpeed!=0.0)	{
					vs = VirtualSpeed;
					VirtualSpeed = 0.0;
					printf("PAUSE!\n");
				} else	{
					VirtualSpeed = vs;
					printf("Continue!\n");
				}						break;
		case 'O':	VirtualSpeed = 1.0;
				printf("VirtualSpeed = %3.2f\n",VirtualSpeed);	break;

		case 'V':	ClearViewVar ();
				printf("VIEW: View var returned.\n");		break;
		case 'U':	Up ();
				printf("Up: Up!\n");				break;
		case ESCAPE:	glutDestroyWindow (WinGod);
				glutDestroyWindow (WinCar);
				DestroySimulation ();
				dCloseODE ();
				exit (0);
		default:	printf("     Unknown command?\n");
	}
}

void SpecialKeyPress (int key, int x, int y)
{
	switch (key) {
		case GLUT_KEY_UP:	KeyState[0] = 1;	break;
		case GLUT_KEY_DOWN:	KeyState[1] = 1;	break;
		case GLUT_KEY_LEFT:	KeyState[2] = 1;	break;
		case GLUT_KEY_RIGHT:	KeyState[3] = 1;	break;
	}
}

void PrintHelp ()
{
	printf(	"\n"VERSION"\n");
	printf(	"KEY\tFUNCTION\n"
		"----------------------------------------------------------------\n"
		"F1\tshow this message\n"
		"1,2...\tchange view type\n"
		"0\tmouse mode\n"
		"P\tpause moniter\n"
		"A\tenable/disable AI driver\n"
		"D\tenable/disable debug mode\n"
		"R\treset simulator\n"
		"T\treset simulator and reverse the track\n"
		"C\tclear the route\n"
		"+\tincress moniter speed\n"
		"-\tdecress moniter speed\n"
		"O\tdefault moniter speed\n"
		"V\tdefault view direction\n"

		"Arrow\tdrive the car(disable the AI)\n"
		"ESC\tExit\n"
		"----------------------------------------------------------------\n\n"
		);
}

void SpecialKeyUp(int key, int x, int y)
{
	switch (key) {
		case GLUT_KEY_UP:	KeyState[0] = 0;	break;
		case GLUT_KEY_DOWN:	KeyState[1] = 0;	break;
		case GLUT_KEY_LEFT:	KeyState[2] = 0;	break;
		case GLUT_KEY_RIGHT:	KeyState[3] = 0;	break;
		case GLUT_KEY_F1:	PrintHelp ();		break;
	}
}

void Mouse (int button, int state, int x, int y)
{
	if (state==GLUT_DOWN)
		switch (button) {
			case GLUT_LEFT_BUTTON:	LeftFlag  = 1;	prex = x; prey = y;	break;
			case GLUT_RIGHT_BUTTON:	RightFlag = 1;	prex = x; prey = y;	break;
			case GLUT_MIDDLE_BUTTON:MiddleFlag= 1;	prex = x; prey = y;	break;
			case GLUT_WHEEL_UP:	distance -= 0.1;	break;
			case GLUT_WHEEL_DOWN:	distance += 0.1;	break;
		}
	else	switch (button) {
			case GLUT_LEFT_BUTTON:	LeftFlag  = 0;	break;
			case GLUT_RIGHT_BUTTON:	RightFlag = 0;	break;
			case GLUT_MIDDLE_BUTTON:MiddleFlag= 0;	break;
		}
}

void motion (int x, int y)	// been call while draging mouse
{
	if (LeftFlag) {
		h += (prex-x) / 10.0;
		pitch  += (prey-y) / 10.0;
	}
	if (RightFlag)	height -= (prey-y) / 500.0;
	if (MiddleFlag) {
		sVector delta(x-prex,prey-y,0);
		delta.RotateZ (-(hpr[0]+h)*DEG_TO_RAD+M_PI_2);
		ViewPointDelta += delta;
	//	ViewPointDelta.print ();
	//	ViewPoint += delta;
	}
	prex = x;
	prey = y;
}

void Reshape (int width, int height)
{
	printf("%d %d\n",width,height);
//	glutReshapeWindow (WINDOW_WIDTH_CUSTOM, WINDOW_HEIGHT_CUSTOM);
}
