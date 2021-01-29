#pragma once
#include <map>
#include <string>
using namespace std;

class AbstractRobot{
public:
    AbstractRobot()=default;
    virtual ~AbstractRobot()=default;
    virtual void setParams(map<string, double>) = 0;
    virtual void oneMoveStep(double velDesired, double angVelDesired, double &x, double &y, double &orient, double &v,double &angV, double &strAng, double dt) = 0;
    virtual AbstractRobot* clone() const = 0;
    virtual double getMaxV() const = 0;
};