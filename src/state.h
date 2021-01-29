#pragma once


// This the class of the state of the robot. The main information about the physical robot are
// the coordination (x,y) and the orientation. Other information is used for implementation of the algorithm.
class State
{
public:
	State();
	~State();
	State (double x, double y, double orient, double gCost, int parent, double v, double angV, double strAngle);
	State (double x, double y, double orient, double gCost, int parent, double v);
	State (double x, double y, double orient, double gCost, int parent);
	State (double x, double y, double orient, double gCost);
	State (double x, double y, double orient);
	State (double x, double y);
	State(const State& s);
    State & operator=(const State &other);
    State(double x, double y, double orient, double gCost, double origX, double origY, double origOrient, double v,
          double angV, double strAngle, int parent);

    double x_;
    double y_;
    double orient_;
    double gCost_;
    double origX_;
    double origY_;
    double origOrient_;
    double v_;
    double angV_;
    double strAngle_;
    int parent_;
};
