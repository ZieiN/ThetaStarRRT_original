//
// Created by zain on 23.01.2021.
//
#include "robotstate.h"



RobotState::RobotState() {
 x_ = 0;
 y_ = 0;
 orientation_ = 0;
 vel_ = 0;
 angVel_ = 0;
 steerAngle_ = 0;
}

RobotState::~RobotState() {
 // default destructor
}

RobotState::RobotState(double x, double y, double orientation, double vel, double angVel, double steerAngle) :
        x_(x),y_(y),orientation_(orientation),vel_(vel),angVel_(angVel),steerAngle_(steerAngle) {}

RobotState::RobotState(const RobotState &other) {
 x_ = other.x_;
 y_ = other.y_;
 orientation_ = other.orientation_;
 steerAngle_ = other.steerAngle_;
 vel_ = other.vel_;
 angVel_ = other.angVel_;
}

double RobotState::getVel() const {
    return vel_;
}

double RobotState::getAngVel() const {
    return angVel_;
}

double RobotState::getX() const {
    return x_;
}

double RobotState::getY() const {
    return y_;
}

double RobotState::getSteerAngle() const {
    return steerAngle_;
}

double RobotState::getOrientation() const {
    return orientation_;
}

void RobotState::setVel(double vel) {
    vel_ = vel;
}

void RobotState::setAngVel(double angVel) {
    angVel_ = angVel;
}

void RobotState::setX(double x) {
    x_ = x;
}

void RobotState::setY(double y) {
    y_ = y;
}

void RobotState::setSteerAngle(double steerAngle) {
    steerAngle_ = steerAngle;
}

void RobotState::setOrientation(double orientation) {
    orientation_ = orientation;
}