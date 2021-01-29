//
// Created by zain on 23.01.2021.
//
#include "posq.h"
#include <cmath>
#include "geometricfunctions.h"
#include "map.h"
#include "vector"
#include "iostream"
using namespace std;


Posq::Posq() {
    // default constructor
}

Posq::~Posq() {
    // default destructor
}

void Posq::setParams(map<string, double> &params, AbstractRobot *robot) {
    kRho_ = params["krho"];
    kPhi_ = params["kphi"];
    kAlpha_ = params["kalpha"];
    kVi_ = params["kvi"];
    dt_ = params["dt"];
    numIterations_ = params["num_iterations"];
    robot_ = robot->clone();
}

RobotState Posq::getLastRobotSate() {
    return foundTrajectory_.back();
}


Posq &Posq::operator=(const Posq &other) {
    cout<<3<<endl;
    if (this != &other) {
        cout<<4<<endl;
        kRho_ = other.kRho_;
        kPhi_ = other.kPhi_;
        kAlpha_ = other.kAlpha_;
        kVi_ = other.kVi_;
        dt_ = other.dt_;
        numIterations_ = other.numIterations_;
        robot_ = other.robot_->clone();
    }
    return *this;
}


bool Posq::steer(const RobotState &s1, const RobotState &s2, const Map &mp, double eps) {
    foundTrajectory_.clear();
    double x = s1.getX(), gX = s2.getX();
    double y = s1.getY(), gY = s2.getY();
    double orient = s1.getOrientation(), gOrient = s2.getOrientation();
    double v = s1.getVel(), gV = s2.getVel();
    double strAng = s1.getSteerAngle(), gStrAng = s2.getSteerAngle();
    double angV = s1.getAngVel(), gAngV = s2.getAngVel();
    double cost = 0;
    double velDesired, angVelDesired, alpha, phi, rho;
    rho = hypot(x-gX, y-gY);
    alpha = minAngle(atan2(gY-y, gX-x), orient);
    phi = minAngle(gOrient, orient);
    int mxIter = numIterations_;
    foundTrajectory_.emplace_back(x, y, orient, v, angV, strAng);
    while(mxIter-- > 0 && rho > eps)
    {
        velDesired = kRho_ * tanh(kVi_ * rho);
        angVelDesired = (kAlpha_ * alpha + kPhi_ * phi)/rho;
        robot_->oneMoveStep(velDesired, angVelDesired, x, y, orient, v, angV, strAng, dt_);
        rho = hypot(x-gX, y-gY);
        alpha = minAngle(atan2(gY-y, gX-x), orient);
        phi = minAngle(gOrient, orient);
        if(mp.cellIsObstacle(x, y)){
            return false;
        }
        cost += dt_;
        foundTrajectory_.emplace_back(x, y, orient, v, angV, strAng);
    }
    pathCost_ = cost;
    return true;
}

vector<RobotState> Posq::getFoundTrajectory() {
    return foundTrajectory_;
}

double Posq::getPathCost() const {
    return pathCost_;
}

double Posq::simulationMaxDistance(){
    return dt_*numIterations_*(robot_->getMaxV());
}

Posq::Posq(const Posq &other) {
    cout<<"2"<<endl;
    kRho_ = other.kRho_;
    kPhi_ = other.kPhi_;
    kAlpha_ = other.kAlpha_;
    kVi_ = other.kVi_;
    dt_ = other.dt_;
    numIterations_ = other.numIterations_;
    robot_ = other.robot_->clone();
}
