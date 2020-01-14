/*
 *	VERSION:	SCS-1.0.1
 *	DATE:		2013.Sep.1
 *	AUTHOR:		Yu Kunlin
 */

#include <ode/ode.h>
#include <GL/freeglut.h>
#include "car.h"
#include "draw.h"
#include "vector.h"

	sObjectID Chassis = NULL, Battery;
	sObjectID Wheel_FL, Wheel_FR, Wheel_BL, Wheel_BR;

	// Joint between wheels and chassis
	dJointID Joint_FL, Joint_FR, Joint_BL, Joint_BR;

	int CarReverseFlag = 0;
	int CarDirection = 1;

	sVector BatteryPos (0,-0.05,0.02);
	sVector BatteryPosR(0,0.05,0.02);
	sVector BatteryPosB(0,-0.01,-0.05);

	double ChassisLength	= 0.26;		// chassis length
	double ChassisWidth	= 0.11;		// chassis width
	double ChassisHeight	= 0.005;	// chassis height
	double ChassisMass	= 0.5;		// chassis mass

	double WheelLR		= 0.135;	// Distance between left and right wheel
	double WheelFront	= 0.08;		// Distance between front wheel and middle
	double WheelBack	= 0.12;		// Distance between back wheel and middle
	double WheelMass	= 0.03;		// wheel mass

	double WheelRadius	= B_WHEEL_RADIUS;	// wheel radius
	double WheelFrontWidth	= B_WHEEL_FRONT_WIDTH;	// wheel width
	double WheelBackWidth	= B_WHEEL_BACK_WIDTH;
	double WheelWidth	= B_WHEEL_FRONT_WIDTH;

	double CarWidth		= 0.11 + B_WHEEL_BACK_WIDTH;

	double BatteryLength	= 0.13;
	double BatteryWidth	= 0.04;
	double BatteryHeight	= 0.02;
	double BatteryMass	= 0.3;

	sVector	InductancePos	[INDUCTANCE_NUM];	// save every inductance's position
	sVector Magnetic	[INDUCTANCE_NUM];	// save the magnetic of every inductance
	int	Ip = 0;					// inductance pointer

void DestroyObject (sObjectID obj) {
	if (!obj) return;
	dBodyDestroy (obj->body);
	dGeomDestroy (obj->geom);
	free (obj);
}

void DestroyCar () {
	DestroyObject (Chassis);
	DestroyObject (Wheel_FL);
	DestroyObject (Wheel_FR);
	DestroyObject (Wheel_BL);
	DestroyObject (Wheel_BR);
	DestroyObject (Battery);
}

void MakeCar (double x, double y, dWorldID world, dSpaceID space)
{
	dMass m;
	dQuaternion q;

	//if (Chassis) DestroyCar ();

	if (cartype == electromagnetic) {
		WheelRadius	= A_WHEEL_RADIUS;	// wheel radius
		WheelFrontWidth	= A_WHEEL_FRONT_WIDTH;	// wheel width
		WheelBackWidth	= A_WHEEL_BACK_WIDTH;
		WheelWidth	= A_WHEEL_FRONT_WIDTH;
		CarWidth	= ChassisWidth + A_WHEEL_BACK_WIDTH;
	}

	// chassis body
	Chassis = (sObjectID)malloc(sizeof(sObject));
	Chassis->body = dBodyCreate (world);
	dBodySetPosition (Chassis->body,x,y,STARTZ);
	dMassSetBoxTotal (&m,ChassisMass,ChassisWidth,ChassisLength,ChassisHeight);
	dBodySetMass (Chassis->body,&m);
	Chassis->geom = dCreateBox (space,ChassisWidth,ChassisLength,ChassisHeight);
	dGeomSetBody (Chassis->geom,Chassis->body);

	// Battery body
	Battery = (sObjectID)malloc(sizeof(sObject));
	Battery->body = dBodyCreate (world);
	if (!CarDirection)
		dBodySetPosition (Battery->body, x+BatteryPos .GetX(), y+BatteryPos .GetY(), STARTZ+BatteryPos .GetZ());
	else	dBodySetPosition (Battery->body, x+BatteryPosR.GetX(), y+BatteryPosR.GetY(), STARTZ+BatteryPosR.GetZ());
	dMassSetBoxTotal (&m,BatteryMass,BatteryLength,BatteryWidth,BatteryHeight*5);
	dBodySetMass (Battery->body,&m);
	Battery->geom = dCreateBox (space,BatteryLength,BatteryWidth,BatteryHeight);
	dGeomSetBody (Battery->geom,Battery->body);

	// Battery joint
	dJointID JointChassisBattery = dJointCreateFixed(world,0);
	dJointAttach (JointChassisBattery,Chassis->body,Battery->body);
	dJointSetFixed (JointChassisBattery);

	// wheel bodies
#define WHEEL(WHICH); 								\
	Wheel_##WHICH = (sObjectID)malloc(sizeof(sObject));			\
	Wheel_##WHICH->body = dBodyCreate (world);				\
	dQFromAxisAndAngle (q,0,1,0,M_PI/2.0);					\
	dBodySetQuaternion (Wheel_##WHICH->body,q);				\
	dMassSetCylinder (&m,1,1,WheelRadius,WheelWidth);			\
	dMassAdjust (&m,WheelMass);						\
	dBodySetMass (Wheel_##WHICH->body,&m);					\
	Wheel_##WHICH->geom = dCreateCylinder (space,WheelRadius,WheelWidth);	\
	dGeomSetBody (Wheel_##WHICH->geom,Wheel_##WHICH->body);

	WheelWidth = WheelFrontWidth;
	WHEEL(FL);
	WHEEL(FR);
	WheelWidth = WheelBackWidth;
	WHEEL(BL);
	WHEEL(BR);
#undef WHEEL

	if (!CarDirection) {
		dBodySetPosition (Wheel_FL->body, x-0.5*WheelLR, y+WheelFront, STARTZ+0.01);
		dBodySetPosition (Wheel_FR->body, x+0.5*WheelLR, y+WheelFront, STARTZ+0.01);
		dBodySetPosition (Wheel_BL->body, x-0.5*WheelLR, y-WheelBack , STARTZ+0.01);
		dBodySetPosition (Wheel_BR->body, x+0.5*WheelLR, y-WheelBack , STARTZ+0.01);
	}
	else {
		dBodySetPosition (Wheel_BL->body, x-0.5*WheelLR, y+WheelBack , STARTZ+0.01);
		dBodySetPosition (Wheel_BR->body, x+0.5*WheelLR, y+WheelBack , STARTZ+0.01);
		dBodySetPosition (Wheel_FL->body, x-0.5*WheelLR, y-WheelFront, STARTZ+0.01);
		dBodySetPosition (Wheel_FR->body, x+0.5*WheelLR, y-WheelFront, STARTZ+0.01);
	}


	// front wheel hinge2s
#define JOINT_F(WHICH);										\
	Joint_F##WHICH = dJointCreateHinge2 (world,0);						\
	dJointAttach (Joint_F##WHICH, Chassis->body, Wheel_F##WHICH->body);			\
	const double *pos_F##WHICH = dBodyGetPosition (Wheel_F##WHICH->body);			\
	dJointSetHinge2Anchor(Joint_F##WHICH,pos_F##WHICH[0],pos_F##WHICH[1],pos_F##WHICH[2]);	\
	dJointSetHinge2Axis1 (Joint_F##WHICH,0,-0.1,1);						\
	dJointSetHinge2Axis2 (Joint_F##WHICH,1,0,0);						\
	dJointSetHinge2Param (Joint_F##WHICH,dParamFMax,FMAX);					\
	dJointSetHinge2Param (Joint_F##WHICH,dParamFMax2,0);					\
	dJointSetHinge2Param (Joint_F##WHICH,dParamVel,0);					\
	dJointSetHinge2Param (Joint_F##WHICH,dParamSuspensionERP,0.8);				\
	dJointSetHinge2Param (Joint_F##WHICH,dParamSuspensionCFM,1e-5);				

	JOINT_F(L);
	JOINT_F(R);
#undef JOINT_F

	// back wheel hinges
#define JOINT_B(WHICH);										\
	Joint_B##WHICH = dJointCreateHinge (world,0);						\
	dJointAttach (Joint_B##WHICH, Chassis->body, Wheel_B##WHICH->body);			\
	const double *pos_B##WHICH = dBodyGetPosition(Wheel_B##WHICH->body);			\
	dJointSetHingeAnchor(Joint_B##WHICH,pos_B##WHICH[0],pos_B##WHICH[1],pos_B##WHICH[2]);	\
	dJointSetHingeAxis  (Joint_B##WHICH,1,0,0);						\
	dJointSetHingeParam (Joint_B##WHICH,dParamFMax,FMAX);					\
	dJointSetHingeParam (Joint_B##WHICH,dParamVel,0);					\
	dJointSetHingeParam (Joint_B##WHICH,dParamSuspensionERP,0.8);				\
	dJointSetHingeParam (Joint_B##WHICH,dParamSuspensionCFM,1e-5);

	JOINT_B(L);
	JOINT_B(R);
#undef JOINT_B
}


void MakeBalanceCar (double x, double y, dWorldID world, dSpaceID space) {
	dMass m;
	dQuaternion q;

	// chassis body
	Chassis = (sObjectID)malloc(sizeof(sObject));
	Chassis->body = dBodyCreate (world);
	dQFromAxisAndAngle (q,1,0,0,M_PI/2.0);
	dBodySetQuaternion (Chassis->body,q);
	dBodySetPosition (Chassis->body,x,y,STARTZ+0.1);
	dMassSetBoxTotal (&m,ChassisMass/2.0,ChassisWidth,ChassisLength,ChassisHeight);
	dBodySetMass (Chassis->body,&m);
	Chassis->geom = dCreateBox (space,ChassisWidth,ChassisLength,ChassisHeight);
	dGeomSetBody (Chassis->geom,Chassis->body);

	// Battery body
	Battery = (sObjectID)malloc(sizeof(sObject));
	Battery->body = dBodyCreate (world);
	dQFromAxisAndAngle (q,1,0,0,M_PI/2.0);
	dBodySetQuaternion (Battery->body,q);
	dBodySetPosition (Battery->body, x+BatteryPosB.GetX(), y+BatteryPosB.GetY(), STARTZ+BatteryPosB.GetZ()+0.1);
	dMassSetBoxTotal (&m,BatteryMass,BatteryLength,BatteryWidth,BatteryHeight*5);
	dBodySetMass (Battery->body,&m);
	Battery->geom = dCreateBox (space,BatteryLength,BatteryWidth,BatteryHeight);
	dGeomSetBody (Battery->geom,Battery->body);

	// Battery hinge
	dJointID j = dJointCreateFixed(world,0);
	dJointAttach (j,Chassis->body,Battery->body);
	dJointSetFixed (j);


	// wheel bodies
#define WHEEL(WHICH); 								\
	Wheel_##WHICH = (sObjectID)malloc(sizeof(sObject));			\
	Wheel_##WHICH->body = dBodyCreate (world);				\
	dQFromAxisAndAngle (q,0,1,0,M_PI/2.0);					\
	dBodySetQuaternion (Wheel_##WHICH->body,q);				\
	dMassSetCylinder (&m,1,1,WheelRadius,WheelWidth);			\
	dMassAdjust (&m,WheelMass);						\
	dBodySetMass (Wheel_##WHICH->body,&m);					\
	Wheel_##WHICH->geom = dCreateCylinder (space,WheelRadius,WheelWidth);	\
	dGeomSetBody (Wheel_##WHICH->geom,Wheel_##WHICH->body);

	WheelWidth = WheelBackWidth;
	WHEEL(BL);
	WHEEL(BR);
#undef WHEEL

	dBodySetPosition (Wheel_BL->body, x-0.5*WheelLR, y, STARTZ+0.1-WheelBack);
	dBodySetPosition (Wheel_BR->body, x+0.5*WheelLR, y, STARTZ+0.1-WheelBack);

	// back wheel hinges
#define JOINT_B(WHICH);										\
	Joint_B##WHICH = dJointCreateHinge (world,0);						\
	dJointAttach (Joint_B##WHICH, Chassis->body, Wheel_B##WHICH->body);			\
	const double *pos_B##WHICH = dBodyGetPosition(Wheel_B##WHICH->body);			\
	dJointSetHingeAnchor(Joint_B##WHICH,pos_B##WHICH[0],pos_B##WHICH[1],pos_B##WHICH[2]);	\
	dJointSetHingeAxis  (Joint_B##WHICH,1,0,0);						\
	dJointSetHingeParam (Joint_B##WHICH,dParamFMax,FMAX);					\
	dJointSetHingeParam (Joint_B##WHICH,dParamVel,0);					\
	dJointSetHingeParam (Joint_B##WHICH,dParamSuspensionERP,0.8);				\
	dJointSetHingeParam (Joint_B##WHICH,dParamSuspensionCFM,1e-5);

	JOINT_B(L);
	JOINT_B(R);
#undef JOINT_B
}

// The definition of matrix between ode & opengl are diferrent.
static void TransformMatrix (double matrix[16],sVector pos, const double R[12])
{
	matrix[0]  = R[0];
	matrix[1]  = R[4];
	matrix[2]  = R[8];
	matrix[3]  = 0;
	matrix[4]  = R[1];
	matrix[5]  = R[5];
	matrix[6]  = R[9];
	matrix[7]  = 0;
	matrix[8]  = R[2];
	matrix[9]  = R[6];
	matrix[10] = R[10];
	matrix[11] = 0;
	matrix[12] = pos.GetX();
	matrix[13] = pos.GetY();
	matrix[14] = pos.GetZ();
	matrix[15] = 1;
}

static void DrawChassis ()
{
	static sVector pp,p,c;
	sVector pos(dGeomGetPosition (Chassis->geom));
	sVector speed (dBodyGetLinearVel (Chassis->body));
	const double * R = dGeomGetRotation (Chassis->geom);
	static double r=1.0,l;
	double matrix[16];

	TransformMatrix (matrix,pos,R);
	glPushMatrix ();
	glMultMatrixd (matrix);
	glTranslated (0.0,0.0,ViewPoint.GetZ()/1000.0);

	glScaled (ChassisWidth,ChassisLength,ChassisHeight);
	glColor3f (0.2f,0.2f,0.3f);
	glutSolidCube (1);

	glPopMatrix ();
}

static void DrawBattery ()
{
	sVector pos(dGeomGetPosition (Battery->geom));
	const double * R = dGeomGetRotation (Battery->geom);
	GLdouble matrix[16];

	glColor3f (0.6f,0.6f,0.9f);

	TransformMatrix (matrix,pos,R);
	glPushMatrix ();
	glMultMatrixd (matrix);

	glScaled (BatteryLength,BatteryWidth,BatteryHeight);
	glutSolidCube (1);

	glPopMatrix ();
}


static void DrawWheel ()
{
	sVector pos;
	const double * R;
	double matrix[16];
	sVector speed;
	double l;

#define WHEEL(WHICH)					\
	pos.set(dGeomGetPosition(Wheel_##WHICH->geom));	\
	speed.set(dBodyGetLinearVel(Wheel_##WHICH->body));	\
		l = speed.GetLen ()/10.0*2.5;								\
		R	= dGeomGetRotation(Wheel_##WHICH->geom);\
	TransformMatrix(matrix,pos,R);			\
	glPushMatrix();					\
	glTranslated (0.0,0.0,ViewPoint.GetZ()/1000.0);	\
	glMultMatrixd (matrix);				\
	glTranslated (0.0f,0.0f,-WheelWidth/2.0);	\
	glColor3d (0.4f,0.2f,0.2f);			\
	glLineWidth (1);				\
glEnable(GL_POLYGON_OFFSET_FILL);			\
glPolygonOffset(1.0f, 1.0f);				\
	glutSolidCylinder(WheelRadius,WheelWidth,18,4);\
glDisable(GL_POLYGON_OFFSET_FILL);			\
	glColor3f (0.3f,0.1f,0.1f);			\
	glutWireCylinder(WheelRadius,WheelWidth,18,4);\
	glPopMatrix();

	WheelWidth = WheelFrontWidth;
	WHEEL(FL);
	WHEEL(FR);
	WheelWidth = WheelBackWidth;
	WHEEL(BL);
	WHEEL(BR);
#undef WHEEL
}


static void DrawBalanceWheel ()
{
	sVector pos;
	const double * R;
	double matrix[16];

	glColor3f (0.9f,0.2f,0.2f);

#define WHEEL(WHICH)					\
	pos.set(dGeomGetPosition(Wheel_##WHICH->geom));	\
	R	= dGeomGetRotation(Wheel_##WHICH->geom);\
	TransformMatrix(matrix,pos,R);			\
	glPushMatrix();					\
	glTranslated (0.0,0.0,ViewPoint.GetZ()/1000.0);	\
	glMultMatrixd (matrix);				\
	glTranslated(0.0f,0.0f,-WheelWidth/2.0);	\
	glColor3f (0.4f,0.2f,0.2f);			\
	glLineWidth (1);				\
glEnable(GL_POLYGON_OFFSET_FILL);			\
glPolygonOffset(1.0f, 1.0f);				\
	glutSolidCylinder(WheelRadius,WheelWidth,18,4);\
glDisable(GL_POLYGON_OFFSET_FILL);			\
	glColor3f (0.3f,0.1f,0.1f);			\
	glutWireCylinder(WheelRadius,WheelWidth,18,4);\
	glPopMatrix();

	WHEEL(BL);
	WHEEL(BR);
#undef WHEEL
}

static void DrawInductance ()
{
	sVector pos(dGeomGetPosition (Chassis->geom));
	const double * R = dGeomGetRotation (Chassis->geom);
	double matrix[16];

	TransformMatrix (matrix,pos,R);
	glPushMatrix ();
	glMultMatrixd (matrix);

	for (int i=0; i<Ip; i++) {
		glPushMatrix ();
		glTranslated (InductancePos[i].GetX(),InductancePos[i].GetY(),InductancePos[i].GetZ());

		glColor3f (0.9f,0.2f,0.2f);
		glLineWidth (2);
		Magnetic[i] /= 100.0;
		glBegin (GL_LINES);
		glVertex3d (0,0,0);
		glVertex3d (Magnetic[i].GetX(),Magnetic[i].GetY(),Magnetic[i].GetZ());
		glEnd ();

		glColor3f (0.1f,0.1f,0.2f);
		glutSolidSphere (0.005f,10,10);
		glPopMatrix ();
	}
	glPopMatrix ();
}

void DrawCar ()
{
	DrawChassis ();
	DrawBattery ();
	DrawWheel ();
	DrawInductance ();
}

void DrawBalanceCar ()
{
	DrawChassis ();
	DrawBattery ();
	DrawBalanceWheel ();
}

sVector CarX ()
{
	sVector Left (dBodyGetPosition(Wheel_BL->body));
	sVector Right(dBodyGetPosition(Wheel_BR->body));

	return (Right - Left).Normalize();
}

sVector CarY ()
{
	if (cartype != balance) {
		sVector PosF(dBodyGetPosition (Wheel_FL->body));
		sVector PosB(dBodyGetPosition (Wheel_BL->body));
		sVector dir = PosF - PosB;
		dir.Normalize ();
		if (CarDirection) dir *= -1;
		return dir;
	} else {
		sVector PosL(dBodyGetPosition (Wheel_BL->body));
		sVector PosR(dBodyGetPosition (Wheel_BR->body));
		sVector PosM(dBodyGetPosition (Chassis->body));
		sVector PosB = (PosL + PosR) / 2;
		sVector Front = PosM - PosB;
		sVector Y = VMult (Front,PosR-PosL);
		Y.Normalize ();
		return Y;
	}
}

sVector CarZ ()
{
	return VMult (CarX(),CarY()).Normalize();
}

sVector ToCarCoo (sVector v)
{
	sVector v1;
	v1.SetX (NMult(v,CarX()));
	v1.SetY (NMult(v,CarY()));
	v1.SetZ (NMult(v,CarZ()));
	return v1;
}

sVector ToWorldCoo (sVector v)
{
	sVector v1 = v.GetX()*CarX() + v.GetY()*CarY() + v.GetZ()*CarZ();
	return v1;
}
