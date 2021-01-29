#pragma once
#include "robot.h"
#include <map>
#include <string>

class DDRobot : public AbstractRobot
{
public:
	DDRobot();
	~DDRobot() override;
    DDRobot &operator=(const DDRobot &other);
    DDRobot(const DDRobot &other);
	void setParams(map<string, double> param) override;
	DDRobot* clone() const override;
    void oneMoveStep(double velDesired, double angVelDesired, double &x, double &y, double &orient, double &v,double &angV, double &strAng, double dt);
	double getMaxV() const;
    double getMaxAcc() const;
    double getMaxOmega() const;
    double getWheelBase() const;
    void setMaxV(double maxV);
    void setMaxAcc(double maxAcc);
    void setMaxOmega(double maxOmega);
    void setWheelBase(double wheelBase);
private:
	double maxV_, maxAcc_, wheelBase_;
};