#include "state.h"

State::State() {
    // default constructor
}

State::~State() {
    // default destructor
}

State::State(double x, double y, double orient, double gCost, int parent, double v, double angV, double strAngle) {
    x_ = x;
    y_ = y;
    orient_ = orient;
    gCost_ = gCost;
    parent_ = parent;
    v_ = v;
    angV_ = angV;
    strAngle_ = strAngle;
}

State::State (double x, double y, double orient, double gCost, int parent, double v){
	x_ = x;
	y_ = y;
	orient_ = orient;
	gCost_ = gCost;
	parent_ = parent;
	v_ = v;
}
State::State (double x, double y, double orient, double gCost, int parent){
	x_ = x;
	y_ = y;
	orient_ = orient;
	gCost_ = gCost;
	parent_ = parent;
}
State::State (double x, double y, double orient, double gCost){
	x_ = x;
	y_ = y;
	orient_ = orient;
	gCost_ = gCost;
}
State::State (double x, double y, double orient){
	x_ = x;
	y_ = y;
	orient_ = orient;
}
State::State (double x, double y){
	x_ = x;
	y_ = y;
}
State::State(const State& other)
{
	x_ = other.x_;
	y_ = other.y_;
	gCost_ = other.gCost_;
	orient_ = other.orient_;
	parent_ = other.parent_;
	origX_ = other.origX_;
	origY_ = other.origY_;
	origOrient_ = other.origOrient_;
	v_ = other.v_;
	angV_ = other.angV_;
	strAngle_ = other.strAngle_;
}

State::State(double x, double y, double orient, double gCost, double origX, double origY, double origOrient, double v,
             double angV, double strAngle, int parent) : x_(x), y_(y), orient_(orient), gCost_(gCost), origX_(origX),
                                                         origY_(origY), origOrient_(origOrient), v_(v), angV_(angV),
                                                         strAngle_(strAngle), parent_(parent) {}

State &State::operator=(const State &other) {
    if (this != &other)
    {
        x_ = other.x_;
        y_ = other.y_;
        gCost_ = other.gCost_;
        orient_ = other.orient_;
        parent_ = other.parent_;
        origX_ = other.origX_;
        origY_ = other.origY_;
        origOrient_ = other.origOrient_;
        v_ = other.v_;
        angV_ = other.angV_;
        strAngle_ = other.strAngle_;
    }
    return *this;
}
