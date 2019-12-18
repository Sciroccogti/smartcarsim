/*
 *	VERSION:	SCS-1.0.1
 *	DATE:		2013.Sep.1
 *	AUTHOR:		Yu Kunlin
 */

#ifndef _VECTOR_H_
#define _VECTOR_H_

class sVector {
private:
	double x;
	double y;
	double z;
	double len;
public:
	sVector ();
	sVector (double x, double y, double z);
	sVector (const double *v);
	double GetX ();
	double GetY ();
	double GetZ ();
	void SetX (double x);
	void SetY (double y);
	void SetZ (double z);
	void set (double x, double y, double z);
	void set (const double *v);
	double CalcLen ();
	double GetLen ();
	sVector Normalize ();
	sVector operator+(sVector v);
	sVector operator-(sVector v);
	sVector operator*(double n);
	friend sVector operator * (double n, sVector v);
	sVector operator / (double div);
	sVector operator += (sVector v);
	sVector operator -= (sVector v);
	sVector operator *= (double n);
	sVector operator /= (double n);
	sVector RotateZ (double theta);
	bool operator == (sVector v);
	bool operator != (sVector v);
	double DirDeg();
	double DirRad();
	void print();
	friend sVector VMult (sVector v1, sVector v2);
	friend double NMult (sVector v1, sVector v2);
};

sVector VMult(sVector v1, sVector v2);
double NMult(sVector v1, sVector v2);
double Distance (sVector v1, sVector v2);
sVector ProjectionXY (sVector v);

#endif
