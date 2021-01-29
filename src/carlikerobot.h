#pragma once
#include "robot.h"
#include <map>
#include <string>

class CarLikeRobot : public AbstractRobot
{
public:
	CarLikeRobot();
	~CarLikeRobot() override;
	CarLikeRobot(const CarLikeRobot &other);
	void setParams(map<string, double> param) override;
	CarLikeRobot* clone() const override;
    void oneMoveStep(double velDesired, double angVelDesired, double &x, double &y, double &orient, double &v,double &angV, double &strAng, double dt);
	double getMaxV() const;
    double getMaxAcc() const;
    double getMaxSteerAngle() const;
    double getMaxOmega() const;
    double getWheelBase() const;
    void setMaxV(double maxV);
    void setMaxAcc(double maxAcc);
    void setMaxSteerAngle(double maxSteerAngle);
    void setMaxOmega(double maxOmega);
    void setWheelBase(double wheelBase);

private:
	double maxV_, maxAcc_, maxSteerAngle_, maxOmega_, wheelBase_;
};