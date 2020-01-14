/*
 *	VERSION:	SCS-1.0.1
 *	DATE:		2013.Sep.1
 *	AUTHOR:		Yu Kunlin
 */

#include <stdio.h>
#include <malloc.h>
#include <math.h>

#include <ode/ode.h>

#include "common.h"
#include "vector.h"

sVector::sVector () {
	x = 0.0;
	y = 0.0;
	z = 0.0;
	CalcLen ();
}

sVector::sVector (double x, double y, double z) {
	this->x = x;
	this->y = y;
	this->z = z;
	CalcLen ();
}

sVector::sVector (const double *v) {
	this->x = v[0];
	this->y = v[1];
	this->z = v[2];
}

double sVector::GetX () {	return this->x;	}
double sVector::GetY () {	return this->y; }
double sVector::GetZ () {	return this->z;	}

void sVector::SetX (double x) {	this->x = x;	}
void sVector::SetY (double y) {	this->y = y;	}
void sVector::SetZ (double z) {	this->z = z;	}

void sVector::set (double x, double y, double z) {
	this->x = x;
	this->y = y;
	this->z = z;
}

void sVector::set (const double *v) {
	this->x = v[0];
	this->y = v[1];
	this->z = v[2];
}

double sVector::CalcLen () {	return len = sqrt(x*x + y*y + z*z);	}
double sVector::GetLen () {	return CalcLen ();	}

sVector sVector::Normalize() {
	if (CalcLen ()==0.0) {
		printf("VECTOR: Normalize zero vector.\n");
		set(1,0,0);
	}
	else	*this /= len;
	return *this;
}

sVector sVector::operator + (sVector v) {
	sVector v1;
	v1.SetX (this->x + v.GetX());
	v1.SetY (this->y + v.GetY());
	v1.SetZ (this->z + v.GetZ());
	return v1;
}

sVector sVector::operator - (sVector v) {
	sVector v1;
	v1.SetX (this->x - v.GetX());
	v1.SetY (this->y - v.GetY());
	v1.SetZ (this->z - v.GetZ());
	return v1;
}

sVector sVector::operator * (double n) {
	sVector v;
	v.SetX (this->x * n);
	v.SetY (this->y * n);
	v.SetZ (this->z * n);
	return v;
}

sVector operator * (double n, sVector v) {
	sVector v1;
	v1.SetX (v.GetX() * n);
	v1.SetY (v.GetY() * n);
	v1.SetZ (v.GetZ() * n);
	return v1;
}

sVector sVector::operator / (double div) {
	sVector v;
	v.SetX (this->x / div);
	v.SetY (this->y / div);
	v.SetZ (this->z / div);
	return v;
}

sVector sVector::operator += (sVector v) {
	*this = *this + v;
	return *this;
}

sVector sVector::operator -= (sVector v) {
	*this = *this - v;
	return *this;
}

sVector sVector::operator *= (double n) {
	*this = *this * n;
	return *this;
}

sVector sVector::operator /= (double div) {
	*this = *this / div;
	return *this;
}

bool sVector::operator == (sVector v) {
	sVector v1 = *this - v;
	v1.CalcLen();
	if (v1.len<0.0001)	return true;
	else			return false;
}

bool sVector::operator != (sVector v) {
	return !(*this==v);
}

sVector sVector::RotateZ (double theta) {
	sVector v;
	double Cos = cos(theta);
	double Sin = sin(theta);
	v.SetX (this->x*Cos - this->y*Sin);
	v.SetY (this->x*Sin + this->y*Cos);
	v.SetZ (this->z);
	*this = v;
	return v;
}

double sVector::DirRad () {
	if (this->x==0.0) {
		if (this->y>0.0)	return M_PI_2;
		else if (this->y<0.0)	return -M_PI_2;
		else			return 0;
	}
	double theta = atan(this->y/this->x);
	if (this->x<0)	if (this->y>0)	theta += M_PI;
			else		theta -= M_PI;
	return theta;
}

double sVector::DirDeg () {
	return DirRad() * RAD_TO_DEG;
}

void sVector::print () {
	printf("x:%6.3lf y:%6.3lf z:%6.3lf\n",this->x,this->y,this->z);
}

sVector VMult (sVector v1, sVector v2) {
	sVector v;
	v.SetX (v1.GetY()*v2.GetZ() - v2.GetY()*v1.GetZ());
	v.SetY (v1.GetZ()*v2.GetX() - v2.GetZ()*v1.GetX());
	v.SetZ (v1.GetX()*v2.GetY() - v2.GetX()*v1.GetY());
	return v;
}

double NMult (sVector v1, sVector v2) {
	return (v1.GetX()*v2.GetX() + v1.GetY()*v2.GetY() + v1.GetZ()*v2.GetZ());
}

double Distance (sVector v1, sVector v2){
	sVector v = v1 - v2;
	return v.GetLen ();
}

sVector ProjectionXY (sVector v) {
	sVector v1 = v;
	v1.SetZ (0.0);
	return v1;
}
