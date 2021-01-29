#include <cmath>
#include "ddrobot.h"
#include <iostream>
using namespace std;

DDRobot::DDRobot(){
	// default constructor
}
DDRobot::~DDRobot(){
	// Default destructor
}
DDRobot::DDRobot(const DDRobot &other) : AbstractRobot(other) {
	maxV_ = other.maxV_;
	maxAcc_ = other.maxAcc_;
	wheelBase_ = other.wheelBase_;
}
void DDRobot::setParams(map<string, double> param){
	maxV_ = param["max_velocity"];
	maxAcc_ = param["max_acceleration"];
	wheelBase_ = param["wheel_base"];
}

DDRobot* DDRobot::clone() const{
    return new DDRobot(*this);
}

void DDRobot::oneMoveStep(double velDesired, double angVelDesired, double &x, double &y, double &orient, double &v,
                               double &angV, double &strAng, double dt){
    velDesired /=2.0; // this, to give some amount of flexibility for angVelDesired.
    double vl = v-angV*wheelBase_/2.0;
    double vr = v+angV*wheelBase_/2.0;

    double vlDesired = velDesired-angVelDesired*wheelBase_/2.0;
    double vrDesired = velDesired+angVelDesired*wheelBase_/2.0;

    if(fabs(vlDesired)>maxV_){
        if(vlDesired<0){
            vlDesired=-maxV_;
        }
        else{
            vlDesired=maxV_;
        }
    }
    if(fabs(vlDesired-vl)/dt > maxAcc_){
        if(vl < vlDesired){
            vlDesired = vl + maxAcc_*dt;
        }
        else{
            vlDesired = vl - maxAcc_*dt;
        }
    }
    if(abs(vrDesired)>maxV_){
        if(vrDesired<0){
            vrDesired=-maxV_;
        }
        else{
            vrDesired=maxV_;
        }
    }
    if(fabs(vrDesired-vr)/dt > maxAcc_){
        if(vr < vrDesired){
            vrDesired = vr + maxAcc_*dt;
        }
        else{
            vrDesired = vr - maxAcc_*dt;
        }
    }
    velDesired = (vlDesired + vrDesired)/2.0;
    angVelDesired = (vrDesired - vlDesired)/wheelBase_;

    // for more precision we take the average of the current velocities and the desired ones
    // when calculating the changes in position and orientation.
    double avgVel = (v + velDesired)/2.0;
    double avgAngVel = (angV + angVelDesired)/2.0;
    double newOrient = orient + dt * avgAngVel;
    while(newOrient < -M_PI){
        newOrient += 2*M_PI;
    }
    while(newOrient > M_PI){
        newOrient -= 2*M_PI;
    }
    x = x + avgVel*dt*cos(newOrient);
    y = y + avgVel*dt*sin(newOrient);
    orient = newOrient;
    v = velDesired;
    angV = angVelDesired;
    while(orient < -M_PI){
        orient += 2*M_PI;
    }
    while(orient > M_PI){
        orient -= 2*M_PI;
    }
}

double DDRobot::getMaxV() const{
	return maxV_;
}

double DDRobot::getMaxAcc() const {
    return maxAcc_;
}

double DDRobot::getWheelBase() const {
    return wheelBase_;
}

void DDRobot::setMaxV(double maxV) {
    maxV_ = maxV;
}

void DDRobot::setMaxAcc(double maxAcc) {
    maxAcc_ = maxAcc;
}

void DDRobot::setWheelBase(double wheelBase) {
    wheelBase_ = wheelBase;
}



