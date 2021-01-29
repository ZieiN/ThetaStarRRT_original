//
// Created by Zain on 23.01.2021.
//

#pragma once

class RobotState {
public:
    RobotState();

    ~RobotState();

    RobotState(double x, double y, double orientation, double vel, double angVel, double steerAngle);

    RobotState(const RobotState &other);

    RobotState &operator=(const RobotState &other);

    void setVel(double vel);

    void setAngVel(double angVel);

    void setX(double x);

    void setY(double y);

    void setSteerAngle(double steerAngle);

    void setOrientation(double orientation);

    double getVel() const;

    double getAngVel() const;

    double getX() const;

    double getY() const;

    double getSteerAngle() const;

    double getOrientation() const;

private:
    double x_, y_, orientation_;
    double vel_;
    double angVel_;
    double steerAngle_;
};

