#pragma once
#include <map>
#include <string>
#include <utility>
#include "robot.h"
#include "carlikerobot.h"
#include <vector>
#include "state.h"
#include "map.h"
using namespace std;


class Posq{
public:
	Posq();
	~Posq();
    Posq(const Posq& other);
	void setParams(map<string, double>&params, AbstractRobot *robot);
	Posq* clone();
	pair<bool, State> steer(const Map&, const State &s1, const State &s2);
	pair<bool, State> steerToGoal(const Map&, const State &s1, const State &s2);
	void extractTrajectory(const vector<State>& tree, vector<State>& path, State lastState);
	double distBySteer(const Map&, const State &s1, const State &s2);
	double simulationMaxDistance();
    double getKRho() const;
    double getKAlpha() const;
    double getKBeta() const;
    double getEps() const;
    double getDt() const;
    double getGoalEps() const;
    int getNumIterations() const;
    void setKRho(double kRho);
    void setKAlpha(double kAlpha);
    void setKBeta(double kBeta);
    void setEps(double eps);
    void setDt(double dt);
    void setGoalEps(double goalEps);
    void setNumIterations(int numIterations);
private:
    double kRho_, kAlpha_, kBeta_, eps_, dt_, goalEps_;
    int numIterations_;
	AbstractRobot *robot_;
};