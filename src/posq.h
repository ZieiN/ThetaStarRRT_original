//
// Created by zain on 23.01.2021.
//
#pragma once
#include "robotstate.h"
#include "robot.h"
#include "map.h"
#include "vector"
using namespace std;

class Posq{
public:
    Posq();
    ~Posq();
    Posq(const Posq& other);
    Posq &operator=(const Posq &other);
    void setParams(map<string, double>&params, AbstractRobot *robot);
    bool steer(const RobotState &s1, const RobotState &s2, const Map &mp, double eps);
    double simulationMaxDistance();
    vector<RobotState> getFoundTrajectory();
    RobotState getLastRobotSate();
    double getPathCost() const;
private:
    double kRho_;
    double kAlpha_;
    double kVi_;
    double kPhi_;
    double dt_;
    int numIterations_;
    double pathCost_;
    RobotState lastState_;
    vector<RobotState> foundTrajectory_;
    AbstractRobot *robot_;
};