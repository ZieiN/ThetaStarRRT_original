#pragma once
#include "state.h"
// This function is used to normalize an angle into [-pi,pi].
void makeAngleBtwPiMinusPi(double& ang);

// Return the difference between two orientations.
double distOrient(double orient1, double orient2);

// Return the difference in orientations of two states.
double distOrient(const State& a, const State& b);

double minAngle(double ang1, double ang2);

//Helper function to calculate the angle between three points.
double angle(double x0, double y0, double x1, double y1, double x2, double y2);

// Return euclidean distance between two states.
double dist(const State& x, const State& y);

