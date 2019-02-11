#include "MinitaurShinContact.h"

MinitaurShinContact::MinitaurShinContact(float _l1, float _l2, float _w)
{
	l1 = _l1;
	l2 = _l2;
	w = _w;
}

double MinitaurShinContact::findLc(double q1,double q2,double q1Dot,double q2Dot)
{
	double temp1 = 2.0*l2*fastsqrt(1.0 - (l1*l1*fastsin(q1/2.0 + q2/2.0)*fastsin(q1/2.0 + q2/2.0))/(l2*l2));
	return (l1*q1Dot*fastcos(q1/2.0 + q2/2.0 - asin((l1*sin(q1/2.0 + q2/2.0))/l2)))/(q1Dot*((l1*fastcos(q1/2.0 + q2/2.0))/temp1 + 0.5) + q2Dot*((l1*fastcos(q1/2.0 + q2/2.0))/temp1 - 0.5));
}

void MinitaurShinContact::LcToXY(double q1,double q2, double lc,float &x, float &y)
{
	double temp1 = fastsqrt((2.0*l2*l2 - l1*l1 + l1*l1*cos(q1 + q2))/(l2*l2));
	double sqrt2 = fastsqrt(2.0);
 	x = -(l1*w*fastcos(q2) - l1*w*fastcos(q1) - 4.0*l1*l2*fastsin(q1) + 2.0*l1*lc*fastsin(q1) + 2.0*l1*lc*fastsin(q2) - sqrt2*l2*w*fastcos(q1/2.0 - q2/2.0)*temp1 + 2.0*sqrt2*l2*lc*fastsin(q1/2.0 - q2/2.0)*temp1)/(4.0*l2);
 	y = -(2.0*l1*lc*fastcos(q1) - 4.0*l1*l2*fastcos(q1) - 2.0*l1*lc*fastcos(q2) + l1*w*fastsin(q1) + l1*w*fastsin(q2) + 2.0*sqrt2*l2*lc*fastcos(q1/2.0 - q2/2.0)*temp1 + sqrt2*l2*w*fastsin(q1/2.0 - q2/2.0)*temp1)/(4.0*l2);
 }

void MinitaurShinContact::SPCM(double q1,double q2,double q1Hat,double q2Hat,float &xResult, float &yResult)
{
	double x1,y1,x2,y2;
	qsToXYshin(q1,q2,x1,y1,x2,y2);
	double m = (y2-y1)/(x2-x1);
	double b = (y1-m*x1);
	qsToXYshin(q1Hat,q2Hat,x1,y1,x2,y2);
	double mHat = (y2-y1)/(x2-x1);
	double bHat = (y1-m*x1);
	xResult = (b-bHat)/(m-mHat);
	yResult = (b*mHat-bHat*m)/(m-mHat);
}

void MinitaurShinContact::qsToXYshin(double q1,double q2, double &x1, double &y1,double &x2, double &y2)
{
	double temp1 = q1/2.0 - q2/2.0 + asin((l1*fastsin(q1/2.0 + q2/2.0))/l2);
	double temp2 = (w*fastcos(temp1))/2.0;
	x1 = temp2 + l1*fastsin(q1);
	x2 = temp2 - fastsin(temp1)*(l2) + l1*fastsin(q1);
	y1 = l1*fastcos(q1) - (w*fastsin(temp1))/2.0;
	y2 = l1*fastcos(q1) - fastcos(temp1)*(l2) - (w*fastsin(temp1))/2.0;
}