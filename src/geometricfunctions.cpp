#include "geometricfunctions.h"
#include "state.h"
#include <math.h>
#include <algorithm>
using namespace std;
// This function is used to normalize an angle into [-pi,pi].
void makeAngleBtwPiMinusPi(double& ang)
{
	while(ang > M_PI)
	{
		ang -= 2 * M_PI;
	}
	while(ang < -M_PI)
	{
		ang += 2 * M_PI;
	}
}

// Return the difference between two orientations.
double distOrient(double orient1, double orient2)
{
	while(orient1 < 0)
	{
		orient1 += 2 * M_PI;
	}
	while(orient2 < 0)
	{
		orient2 += 2 * M_PI;
	}
	return min(fabs(orient1 - orient2), 2 * M_PI - fabs(orient1 - orient2));
}

// Minimum angle between two angles.
double minAngle(double ang1, double ang2){
	double tmp = ang1 - ang2;
	makeAngleBtwPiMinusPi(tmp);
	if(fabs(tmp)>2*M_PI-fabs(tmp)){
		if(tmp>0)
			tmp = tmp-2*M_PI;
		else
			tmp = tmp+2*M_PI;
	}
	while(tmp>M_PI)tmp-=2*M_PI;
	while(tmp<-M_PI)tmp+=2*M_PI;
	return tmp;
}

//Helper function to calculate the angle between three points.
double angle(double x0, double y0, double x1, double y1, double x2, double y2){
	double ang1 = atan2(y0-y1, x0-x1);
	double ang2 = atan2(y2-y1, x2-x1);
	return distOrient(ang1, ang2);
}

// Return euclidean distance between two states.
double dist(const State& x, const State& y)
{
	return hypot(x.x_ - y.x_, x.y_ - y.y_);
}

// Return the difference in orientations of two states.
double distOrient(const State& a, const State& b)
{
	double orient1 = a.orient_;
	double orient2 = b.orient_;
	while(orient1 < 0)
	{
		orient1 += 2 * M_PI;
	}
	while(orient2 < 0)
	{
		orient2 += 2 * M_PI;
	}
	return min(fabs(orient1 - orient2), 2 * M_PI - fabs(orient1 - orient2));
}
