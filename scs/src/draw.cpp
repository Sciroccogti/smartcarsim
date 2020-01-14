/*
 *	VERSION:	SCS-1.0.1
 *	DATE:		2013.Sep.1
 *	AUTHOR:		Yu Kunlin
 */

#include <stdio.h>
#include <time.h>

#include <GL/glut.h>
#include <ode/ode.h>

#include "mytime.h"
#include "simulation.h"
#include "key.h"
#include "car.h"
#include "track.h"
#include "common.h"
#include "draw.h"
#include "api.h"
#include "ui.h"

	// window handle
	int WinCustom[CUSTOMWINDOWNUM],WinCar,WinGod;

	ViewType viewtype = bird;
	DriveMode drivemode = ai;
	CarType cartype = camera;
	int RouteFlag = 0;

	// these three var discribe the location of view camera
	double pitch = 45;
	double height = 2.0;
	double h = 0.0;
	static double x=0.0, y=0.0;
	double distance = 1.0;

	// simulate speed of virtual world
	double VirtualSpeed = 1.0;

	int CustomWindowFlag = 0;
	int CustomWindowNum = 0;
	int PathFlag = 0;

	double hpr[3] = {0,30,0};
	sVector ViewPoint(-0.5,0,0.5);

	sVector CameraPos(0.0,0.0,0.3);
	double DepressionAngle = 30;

static void DrawGround ()
{
	glLineWidth (1);
	glColor3d (0.2f,0.2f,0.2f);
	for (int i=-20; i<=20; i++) {	// vertical line
		glBegin (GL_LINES);
		glVertex3d (i,-20,0);
		glVertex3d (i, 20,0);
		glEnd ();
	}
	for (int i=-20; i<=20; i++) {	// horizental line
		glBegin (GL_LINES);
		glVertex3d (-20,i,0);
		glVertex3d ( 20,i,0);
		glEnd ();
	}
}

static void SetViewPoint (double hpr[3], sVector ViewPoint)
{
	glLoadIdentity ();
	// mult inverse matrix of camera place to current matrix
	glRotated (hpr[2],0,0,1);
	glRotated (hpr[1],1,0,0);
	glRotated (hpr[0],0,1,0);
	glRotated (-90,1,0,0);
	glRotated ( 90,0,0,1);
	glTranslated (-ViewPoint.GetX(),-ViewPoint.GetY(),-ViewPoint.GetZ());
}

void ClearViewVar ()
{
	pitch = 45;
	height = 2.0;

	h = 0.0;
	x=0.0;
	y=0.0;
	distance = 1.0;

	ViewPointDelta.set (0.0,0.0,0.0);
}

static void GodView ()	// calc camera place to view car
{
	sVector pos(dBodyGetPosition (Chassis->body));
	sVector lv(dBodyGetLinearVel (Chassis->body));
	sVector dir = CarY ();
	static double speed = 1.0;

	switch (viewtype) {
		case over:	// `~
			hpr[0] = TrackReverseFlag * 180.0;
			hpr[1] = 90.0;
			hpr[2] = 0.0;
			ViewPoint.SetX (ViewPoint.GetX()*5.0/10.0 + pos.GetX()*5.0/10.0);
			ViewPoint.SetY (ViewPoint.GetY()*7.0/10.0 + pos.GetY()*3.0/10.0);
			ViewPoint.SetZ (height * distance*2.0+pos.GetZ());
			break;
		case car:	// 1
			hpr[0] = -dir.DirDeg();
			hpr[1] = DepressionAngle - asin(dir.GetZ())*RAD_TO_DEG;
			hpr[2] = 0.0;
			ViewPoint = ToWorldCoo(CameraPos) + pos;
			break;
		case bird:	// 2
			hpr[0] = TrackReverseFlag * 180.0;
			hpr[1] = pitch;
			hpr[2] = 0.0;
			speed -= 0.5;
			speed = speed*29.0/30.0 + lv.GetLen()/30.0;
			speed += 0.5;
			ViewPoint.SetX (ViewPoint.GetX()*7.0/10.0 + pos.GetX()*3.0/10.0 - cos((hpr[0])*DEG_TO_RAD)*distance*speed/5.0);
			ViewPoint.SetY (ViewPoint.GetY()*7.0/10.0 + pos.GetY()*3.0/10.0 + sin((hpr[0])*DEG_TO_RAD)*distance*speed/5.0);
			ViewPoint.SetZ (height * distance*speed/3.0+pos.GetZ());
			break;
		case birdf:	// 3
			hpr[0] -= lv.GetX()*sin(hpr[0]*DEG_TO_RAD)*VirtualSpeed+lv.GetY()*cos(hpr[0]*DEG_TO_RAD)*VirtualSpeed;
			hpr[1] = pitch;
			hpr[2] = 0.0;
			speed -= 0.5;
			speed = speed*29.0/30.0 + lv.GetLen()/30.0;
			speed += 0.5;
			ViewPoint.SetX (ViewPoint.GetX()*7.0/10.0 + pos.GetX()*3.0/10.0 - cos((hpr[0])*DEG_TO_RAD)*distance*speed/5.0);
			ViewPoint.SetY (ViewPoint.GetY()*7.0/10.0 + pos.GetY()*3.0/10.0 + sin((hpr[0])*DEG_TO_RAD)*distance*speed/5.0);
			ViewPoint.SetZ (height * distance*speed/3.0+pos.GetZ());
			break;
		case hard:	// 4
			hpr[0] = -dir.DirDeg();
			hpr[1] = pitch;
			hpr[2] = 0.0;
			ViewPoint.SetX (pos.GetX() - cos((hpr[0])*DEG_TO_RAD)*distance);
			ViewPoint.SetY (pos.GetY() + sin((hpr[0])*DEG_TO_RAD)*distance);
			ViewPoint.SetZ (height * distance+pos.GetZ());
			break;
		case back:	// 5
			hpr[0] = -dir.DirDeg();
			hpr[1] = pitch/2.0;
			hpr[2] = 0.0;
			ViewPoint.SetX (pos.GetX() - cos((hpr[0])*DEG_TO_RAD)*distance/3.0);
			ViewPoint.SetY (pos.GetY() + sin((hpr[0])*DEG_TO_RAD)*distance/3.0);
			ViewPoint.SetZ (height * distance/8.0+pos.GetZ());
			break;
		case overf:	// 6
			hpr[0] = -dir.DirDeg();
			hpr[1] = 90.0;
			hpr[2] = 0.0;
			ViewPoint = pos;
			ViewPoint.SetZ (height * distance*2.0+pos.GetZ());
			break;
		case soft:	// 7
			hpr[0] -= lv.GetX()*sin(hpr[0]*DEG_TO_RAD)*6+lv.GetY()*cos(hpr[0]*DEG_TO_RAD)*6;
			hpr[1] = pitch;
			hpr[2] = 0.0;
			ViewPoint.SetX (pos.GetX() - cos((hpr[0])*DEG_TO_RAD)*distance);
			ViewPoint.SetY (pos.GetY() + sin((hpr[0])*DEG_TO_RAD)*distance);
			ViewPoint.SetZ (height * distance+pos.GetZ());
			break;

		case mouse:	// 0
			hpr[0] = 0;
			hpr[1] = pitch;
			ViewPoint.SetZ(height);
			break;
	}
	if (viewtype!=car) {
		hpr[0] += h;
		ViewPoint += ViewPointDelta/100.0;
	}
	SetViewPoint (hpr,ViewPoint);
	if (viewtype!=car) {
		hpr[0] -= h;
		ViewPoint -= ViewPointDelta/100.0;
	}
}

static void CarView ()	// set the place of the camera(the camera of the car, not godview camera)
{
	double hpr[3];  
	sVector pos (dBodyGetPosition (Chassis->body));
	sVector dir = CarY() ;
	sVector ViewPoint = ToWorldCoo(CameraPos) + pos;

	hpr[0] = -dir.DirDeg();
	hpr[1] = DepressionAngle - asin(dir.GetZ())*RAD_TO_DEG;
	hpr[2] = 0.0;

	SetViewPoint (hpr,ViewPoint);
}

static void DrawRoute ()
{
	int Length = 5000;
	sRouteID p,q;
	static sRoute car,car_old;
	sVector posl(dBodyGetPosition (Wheel_BL->body));
	sVector posr(dBodyGetPosition (Wheel_BR->body));
	if (VirtualSpeed>0.0) {
		p = head;
		head = (sRouteID)malloc(sizeof(sRoute));
		head->pos = (posl+posr)/2.0;
		head->next = p;
		if (p)
			head->speed = (head->pos - p->pos).GetLen()/VirtualSpeed;
	}
	p = head;
	int i=0;
	glLineWidth(1);
	glBegin (GL_LINE_STRIP);
	while (p->next) {
		q = p->next;
		glColor3d (1.0-i/1000.0,1.0-i/1000.0,1.0-i/1000.0);
		//double v = p->speed*20;
		//glColor3d (v,v,v);
		glVertex3d (p->pos.GetX(),p->pos.GetY(),p->pos.GetZ());
		p = q;
		if (i++>Length) {
			Free (p);
			break ;
		}
	}
	glEnd ();
}

static void DrawSpeed ()
{
	glLoadIdentity ();
	glTranslated (0,0,-5);
	glColor3d (0.9f,0.2f,0.2f);
	double width = 2.7;
	double height = -1.5;
	double RulerWidth = 0.4;

	// draw ruler
	for (int i=-2; i<8; i++) {
		if (i%5==0)	glLineWidth (3);
		else		glLineWidth (1);
		glBegin (GL_LINES);
		glVertex2d (i%5==0?width-RulerWidth-0.1:width-RulerWidth,i/2.0+height);
		glVertex2d (width,i/2.0+height);
		glEnd ();
	}
	glBegin (GL_LINES);
	glVertex2d (width,height-2);
	glVertex2d (width,height+4);
	glEnd ();

	// draw speed
	sVector lv(dBodyGetLinearVel (Chassis->body));

	int temp[2];				// max line widths are different between OSes
	glGetIntegerv(GL_LINE_WIDTH_RANGE,temp);
	if (temp[1]>10) temp[1] = 10;
	glLineWidth ((GLfloat)temp[1]);

	glBegin (GL_LINES);
	glVertex2d (width,height);
	glVertex2d (width,lv.GetLen()/2+height);
	glEnd ();

	static double duty = 0.0;
	duty = duty*0.9 + (MotorDutyL + MotorDutyR)/40.0*0.1;
	glColor3d (0.2f,0.2f,0.9f);
	glBegin (GL_LINES);
	glVertex2d (width-0.2,height);
	glVertex2d (width-0.2,duty+height);
	glEnd ();

	static double speed = 0.0;
	speed = speed*0.9 + sGetSpeed()*0.1;
	glColor3d (0.2f,0.9f,0.2f);
	glBegin (GL_LINES);
	glVertex2d (width-0.1,height);
	glVertex2d (width-0.1,speed+height);
	glEnd ();
}

static void DrawDirection ()
{
	static double dir = ServoDir/2.0;
	glLoadIdentity ();
	glTranslated (0,-1.5,-5);
	
	dir = dir*8.0/10.0+ServoDir/10.0;
	glColor3d (0.2f,0.2f,0.9f);
	glLineWidth (1);
	glBegin (GL_LINES);
	glVertex2d (0,-0.05);
	glVertex2d (0, 0.02);
	glEnd ();

	glLineWidth (10);
	glBegin (GL_LINES);
	glVertex2d (0,0);
	glVertex2d (dir*4.0/100.0*(CarDirection?-1:1),0);
	glEnd ();
}

static void DrawCCDLine ()
{
	glLoadIdentity ();
	glTranslated (0.0,0.0,-1.0);
	glBegin (GL_LINES);
		glVertex2d (10.0,0.0);
		glVertex2d (-10.0,0.0);
	glEnd ();
}

static void DrawGodWindow ()		// main window, you can use every viewtype
{
	glutSetWindow (WinGod);
	if (viewtype==car)
		glClearColor (0.66f,0.66f,0.66f,1.0f);
	else	glClearColor (0.3f,0.3f,0.3f,1.0f);
	glClear (GL_COLOR_BUFFER_BIT);

	GodView ();
	DrawGround ();

	glEnable (GL_DEPTH_TEST);
	glClear (GL_DEPTH_BUFFER_BIT);
	glDepthFunc (GL_LEQUAL);

	DrawTrack ();
	if (viewtype == car) goto finish;

	if (RouteFlag)
		DrawRoute ();
	Free (NULL);

	switch (cartype) {
		case camera:		DrawCar();		break;
		case electromagnetic:	DrawCar();		break;
		case balance:		DrawBalanceCar ();	break;
	}
finish:
	glDisable (GL_DEPTH_TEST);
	if (cartype==balance && viewtype==car)
		DrawCCDLine ();
	DrawSpeed ();
	if (cartype!=balance)
		DrawDirection ();

	glFlush ();
	glutSwapBuffers ();
}
static void DrawCarWindow ()		// mini window, only can use car viewtype
{
	glutSetWindow (WinCar);

	glClearColor (0.66f,0.66f,0.66f,1.0f);
	glClear (GL_COLOR_BUFFER_BIT);

	CarView ();
	DrawGround ();
	
	glEnable (GL_DEPTH_TEST);
	glClear (GL_DEPTH_BUFFER_BIT);
	glDepthFunc (GL_LEQUAL);

	DrawTrack ();
	glDisable (GL_DEPTH_TEST);
	if (cartype==balance)
		DrawCCDLine ();


	glFlush ();
	glutSwapBuffers ();
}


static void Ai ()	// call AI call back function
{
	static int first = 1;
	Ip = 0;
	if (AiFunc)	AiFunc ();
	else if (first) {
		printf ("AI: There is no AI function.\n");
		drivemode = play;
		first = 0;
	}
}

static void DrawCustomWindow ()
{
	for (int i=0; i<CustomWindowNum; i++) {
		glutSetWindow (WinCustom[i]);
		glClear (GL_COLOR_BUFFER_BIT);
		glLoadIdentity (); 
		glColor3d (1.0,1.0,1.0);

		if (DisplayFunc[i])
			DisplayFunc[i] ();

		glFlush ();
		glutSwapBuffers ();
	}
}

static void callback(int value)		// MAIN FRAME OF TOTAL PROGRAM!!!!!
{
	glutTimerFunc(FIELDTIME_M,&callback,0);

	static double ControlTime = 0.0;
	double fTimes = FIELDTIME/STEPTIME;
	int iTimes = (int)fTimes;
	double tail = fTimes - iTimes;

	DrawGodWindow ();

	if (drivemode==play || drivemode==debug) Play ();

	for (int i=0; i<iTimes*VirtualSpeed; i++) {
		step (STEPTIME);			// simulate
		ControlTime += STEPTIME;
		if (ControlTime>=CONTROLTIME) {
			ControlTime -= CONTROLTIME;
			DrawCarWindow ();
			if (drivemode!=play)	Ai ();	// auto dive (according to CarWindow)
			if (CustomWindowFlag)
						DrawCustomWindow ();
		}
	}
	if (tail>0.00001) {
		step (tail*VirtualSpeed);
		ControlTime += tail;
		if (ControlTime>=CONTROLTIME) {
			ControlTime -= CONTROLTIME;
		 	DrawCarWindow ();
			if (drivemode!=play)	Ai ();
			if (CustomWindowFlag)
						DrawCustomWindow ();
		}
	}
}

void DrawInit()
{
	glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);

	// God Window
	int WindowWidthGod = WINDOW_WIDTH_GOD_0;
	int WindowHeightGod = WINDOW_HEIGHT_GOD_0;
	if (CustomWindowNum>2) {
		WindowWidthGod = WINDOW_WIDTH_GOD_1;
		WindowHeightGod = WINDOW_HEIGHT_GOD_1;
	}
	if (CustomWindowNum>4) {
		WindowWidthGod = WINDOW_WIDTH_GOD_2;
		WindowHeightGod = WINDOW_HEIGHT_GOD_2;
	}
	glutInitWindowSize (WindowWidthGod,WindowHeightGod);
	glutInitWindowPosition (WINDOW_POS_X,WINDOW_POS_Y);
	WinGod = glutCreateWindow (VERSION);
	
	glutKeyboardFunc (&Key);
	glutSpecialFunc (&SpecialKeyPress);
	glutSpecialUpFunc (&SpecialKeyUp);
	glutMouseFunc (&Mouse);
	glutMotionFunc (&motion);
	glutReshapeFunc (&Reshape);

	glMatrixMode (GL_PROJECTION);
	gluPerspective (45.0f,(GLdouble)WindowWidthGod/(GLdouble)WindowHeightGod,0.01f,100.0f);
	glMatrixMode (GL_MODELVIEW);


	// Car Window
	glutInitWindowSize (WINDOW_WIDTH_CAR,WINDOW_HEIGHT_CAR);
	glutInitWindowPosition (WINDOW_POS_X+WindowWidthGod+20,WINDOW_POS_Y);
	WinCar = glutCreateWindow ("Car View");

	glutKeyboardFunc (&Key);
	glutSpecialFunc (&SpecialKeyPress);
	glutSpecialUpFunc (&SpecialKeyUp);
	glutReshapeFunc (&Reshape);

	glMatrixMode (GL_PROJECTION);
	gluPerspective (45.0f,4.0/3.0,0.01f,100.0f);
	glMatrixMode (GL_MODELVIEW);

	// Custom Window
	if (CustomWindowFlag)
	if (CustomWindowNum>4) for (int i=0; i<CustomWindowNum; i++) {
		int x = (i+2)%3;
		int y = (i+2)/3;
		char name[] = "Custom Window 00";
		glutInitWindowSize (WINDOW_WIDTH_CUSTOM,WINDOW_HEIGHT_CUSTOM);
		glutInitWindowPosition (WINDOW_POS_X + x*(WINDOW_WIDTH_CUSTOM+20),WINDOW_POS_Y + (y)*WINDOW_HEIGHT_CUSTOM);
		sprintf(name,"Custom Window %d",i);
		WinCustom[i] = glutCreateWindow (name);	

		glutKeyboardFunc (&Key);
		glutSpecialFunc (&SpecialKeyPress);
		glutSpecialUpFunc (&SpecialKeyUp);
	}
	else if (CustomWindowNum>2) for (int i=0; i<CustomWindowNum; i++) {
		int x = (i+5)%3;
		int y = (i+5)/3;
		char name[] = "Custom Window 00";
		glutInitWindowSize (WINDOW_WIDTH_CUSTOM,WINDOW_HEIGHT_CUSTOM);
		glutInitWindowPosition (WINDOW_POS_X + x*(WINDOW_WIDTH_CUSTOM+10),WINDOW_POS_Y + (y)*WINDOW_HEIGHT_CUSTOM);
		sprintf(name,"Custom Window %d",i);
		WinCustom[i] = glutCreateWindow (name);

		glutKeyboardFunc (&Key);
		glutSpecialFunc (&SpecialKeyPress);
		glutSpecialUpFunc (&SpecialKeyUp);
	}
	else for (int i=0; i<CustomWindowNum; i++) {
		char name[] = "Custom Window 00";
		glutInitWindowSize (WINDOW_WIDTH_CUSTOM,WINDOW_HEIGHT_CUSTOM);
		glutInitWindowPosition (WINDOW_POS_X+WindowWidthGod+20,WINDOW_POS_Y + (i+1)*WINDOW_HEIGHT_CUSTOM);
		sprintf(name,"Custom Window %d",i);
		WinCustom[i] = glutCreateWindow (name);

		glutKeyboardFunc (&Key);
		glutSpecialFunc (&SpecialKeyPress);
		glutSpecialUpFunc (&SpecialKeyUp);
	}
	printf("GUI: Glut initialized!\n");

	// All Window have one same timer callback function
	// And it's the main frame of whole program
	glutTimerFunc(FIELDTIME_M,&callback,0);
}
