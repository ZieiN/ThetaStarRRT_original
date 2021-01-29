#include <math.h>
#include "carlikerobot.h"
#include <iostream>
using namespace std;

CarLikeRobot::CarLikeRobot(){
	// default constructor
}
CarLikeRobot::~CarLikeRobot(){
	// Default destructor
}
CarLikeRobot::CarLikeRobot(const CarLikeRobot &other){
	maxV_ = other.maxV_;
	maxAcc_ = other.maxAcc_;
	maxSteerAngle_ = other.maxSteerAngle_;
	maxOmega_ = other.maxOmega_;
	wheelBase_ = other.wheelBase_;
}
CarLikeRobot &CarLikeRobot::operator=(const CarLikeRobot &other){
    if (this != &other)
    {
        maxV_ = other.maxV_;
        maxAcc_ = other.maxAcc_;
        maxSteerAngle_ = other.maxSteerAngle_;
        maxOmega_ = other.maxOmega_;
        wheelBase_ = other.wheelBase_;
    }
    return *this;
}
void CarLikeRobot::setParams(map<string, double> param){
	maxV_ = param["max_velocity"];
	maxAcc_ = param["max_acceleration"];
	maxSteerAngle_ = param["max_steer_angle"];
	maxOmega_ = param["max_omega"];
	wheelBase_ = param["wheel_base"];
}
CarLikeRobot* CarLikeRobot::clone() const{
    return new CarLikeRobot(*this);
}

void CarLikeRobot::oneMoveStep(double velDesired, double angVelDesired, double &x, double &y, double &orient, double &v,
                               double &angV, double &strAng, double dt) {

    if(velDesired > maxV_){
        velDesired = maxV_;
    }
    else if(velDesired < -maxV_){
        velDesired = -maxV_;
    }
    if((velDesired-v)/dt > maxAcc_){
        velDesired = v + dt*maxAcc_;
    }
    else if((velDesired-v)/dt < -maxAcc_){
        velDesired = v - dt*maxAcc_;
    }
    double strAngDesired = atan(angVelDesired * wheelBase_ / velDesired);
    if(strAngDesired > maxSteerAngle_){
        strAngDesired = maxSteerAngle_;
    }
    else if(strAngDesired < -maxSteerAngle_){
        strAngDesired = -maxSteerAngle_;
    }
    if((strAngDesired-strAng)/dt > maxOmega_){
        strAngDesired = strAng + dt*maxOmega_;
    }
    else if((strAngDesired-strAng)/dt < -maxOmega_){
        strAngDesired = strAng - dt*maxOmega_;
    }
    strAng = strAngDesired;
    double tmp = tan(strAng)/wheelBase_;
    x = x + (cos(orient)*(velDesired+v)/2.0)*dt; // x
    y = y + (sin(orient)*(velDesired+v)/2.0)*dt; // y
    orient = orient + (tmp*(velDesired+v)/2.0)*dt; // orient;
    v = velDesired;
    while(orient < -M_PI){
        orient += 2*M_PI;
    }
    while(orient > M_PI){
        orient -= 2*M_PI;
    }
}

double CarLikeRobot::getMaxV() const{
	return maxV_;
}

double CarLikeRobot::getMaxAcc() const {
    return maxAcc_;
}

double CarLikeRobot::getMaxSteerAngle() const {
    return maxSteerAngle_;
}

double CarLikeRobot::getMaxOmega() const {
    return maxOmega_;
}

double CarLikeRobot::getWheelBase() const {
    return wheelBase_;
}

void CarLikeRobot::setMaxV(double maxV) {
    maxV_ = maxV;
}

void CarLikeRobot::setMaxAcc(double maxAcc) {
    maxAcc_ = maxAcc;
}

void CarLikeRobot::setMaxSteerAngle(double maxSteerAngle) {
    maxSteerAngle_ = maxSteerAngle;
}

void CarLikeRobot::setMaxOmega(double maxOmega) {
    maxOmega_ = maxOmega;
}

void CarLikeRobot::setWheelBase(double wheelBase) {
    wheelBase_ = wheelBase;
}


