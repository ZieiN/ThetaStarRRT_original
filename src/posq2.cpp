#include "posq.h"
#include "state.h"
#include "geometricfunctions.h"
#include <vector>
#include <cmath>
#include <map>
#include <string>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <string>
#include "carlikerobot.h"
#include "map.h"
#include "readconfigfunctions.h"
using namespace std;

Posq::Posq(){
	//default constructor
}

Posq::~Posq(){
    if(robot_)
        delete robot_;
}

void Posq::setParams(map<string, double> &params, AbstractRobot *robot){
	kRho_ = params["krho"];
	kBeta_ = params["kbeta"];
	kAlpha_ = params["kalpha"];
	eps_ = params["EPS"];
	goalEps_ = params["goal_EPS"];
	dt_ = params["dt"];
	numIterations_ = params["num_iterations"];
	robot_ = robot->clone();
}

// This function is our implementation of function 'steer' in pseudo-code in the original article.
// In principle, this function implements POSQ function to steer from state (x,y,orientation)
// to another state (x,y,orientation).
pair<bool, State> Posq::steer(const Map &mp, const State &s1, const State &s2)
{
    bool pathFound =
	double x = s1.x_;
	double y = s1.y_;
	double orient = s1.orient_;
	double v = s1.v_;
	double steerAngle = s1.strAngle_;
	double gCost = s1.gCost_;
	pair<bool, State> ans;
	ans.first = false;
	double vDesired, steerAngleDesired, gamma, alpha, beta, rho;
	rho = hypot(x-s2.x_, y-s2.y_);
	alpha = minAngle(atan2(s2.y_-y, s2.x_-x), orient);
	beta = minAngle(s2.orient_, orient);
	int mxIter = numIterations_;
	while(mxIter-- > 0 && rho > eps_)
	{
		vDesired = kRho_ * rho;
		gamma = (kAlpha_ * alpha + kBeta_ * beta)/rho;
		steerAngleDesired = atan(gamma);
		robot_->oneMoveStep(vDesired, steerAngleDesired, v, steerAngle, x, y, orient, dt_);
		rho = hypot(x-s2.x_, y-s2.y_);
		alpha = minAngle(atan2(s2.y_-y, s2.x_-x), orient);
		beta = minAngle(s2.orient_, orient);

		if(mp.cellIsObstacle(x, y)){
			return ans;
		}
		gCost += dt_;
	}
	ans.first = true;
	ans.second = State(x, y, orient, gCost);
	ans.second.v_ = v;
	return ans;
}

// We added specific function 'steer' to steer to goal state. The difference is that here we need to arrive to the goal state 
// precisely (with eps precision), not the last point in the steer path to the state like in normal 'steer' function.
pair<bool, State> Posq::steerToGoal(const Map &mp, const State &s1, const State &s2)
{
	double x = s1.x_;
	double y = s1.y_;
	double orient = s1.orient_;
	double v = s1.v_;
	double steerAngle = s1.strAngle_;
	double gCost = s1.gCost_;
	pair<bool, State> ans;
	ans.first = false;
	double vDesired, steerAngleDesired, gamma, alpha, beta, rho;
	rho = hypot(x-s2.x_, y-s2.y_);
	alpha = minAngle(atan2(s2.y_-y, s2.x_-x), orient);
	beta = minAngle(s2.orient_, orient);
	int mxIter = numIterations_;
	while(mxIter-- > 0 && rho > goalEps_)
	{
		vDesired = kRho_ * rho;
		gamma = (kAlpha_ * alpha + kBeta_ * beta)/rho;
		steerAngleDesired = atan(gamma);
		robot_->oneMoveStep(vDesired, steerAngleDesired, v, steerAngle, x, y, orient, dt_);
		if(mp.cellIsObstacle(x, y)){
			return ans;
		}
		rho = hypot(x-s2.x_, y-s2.y_);
		alpha = minAngle(atan2(s2.y_-y, s2.x_-x), orient);
		beta = minAngle(s2.orient_, orient);
		gCost += dt_;
	}
	ans.first = true;
	ans.second = State(x, y, orient, gCost, -1, v, steerAngle);
	return ans;
}

// Given the sampling tree and the main points (states) of the resulting path, this function extracts all points of the path.
void Posq::extractTrajectory(const vector<State>& tree, vector<State>& path, State lastState)
{
	vector<State> tmp;
	tmp.push_back(lastState);
	while(lastState.parent_ != -1)
	{
		lastState = tree[lastState.parent_];
		tmp.push_back(lastState);
	}
	reverse(tmp.begin(), tmp.end());
	State s1 = tmp[0];
	double x = s1.x_;
	double y = s1.y_;
	double orient = s1.orient_;
	double v = s1.v_;
	double steerAngle = s1.strAngle_;
	double gCost = s1.gCost_;
	double rho, alpha, beta, vDesired, steerAngleDesired, gamma;
	for(int i = 1; i+1 < tmp.size(); ++i)
	{
		path.emplace_back(x, y, orient, gCost);
		State s2 = tmp[i];
		rho = hypot(x-s2.origX_, y-s2.origY_);
		alpha = minAngle(atan2(s2.origY_-y, s2.origX_-x), orient);
		beta = minAngle(s2.origOrient_, orient);
		int mxIter = numIterations_;
		while(mxIter-- > 0 && rho > eps_)
		{
			vDesired = kRho_ * rho;
			gamma = (kAlpha_ * alpha + kBeta_ * beta)/rho;
			steerAngleDesired = atan(gamma);
			robot_->oneMoveStep(vDesired, steerAngleDesired, v, steerAngle, x, y, orient, dt_);
			rho = hypot(x-s2.origX_, y-s2.origY_);
			alpha = minAngle(atan2(s2.origY_-y, s2.origX_-x), orient);
			beta = minAngle(s2.origOrient_, orient);
			gCost += dt_;
			path.emplace_back(x, y, orient, gCost);
		}
	}
	State s2 = tmp.back();
	rho = hypot(x-s2.origX_, y-s2.origY_);
	alpha = minAngle(atan2(s2.origY_-y, s2.origX_-x), orient);
	beta = minAngle(s2.origOrient_, orient);
	int mx_iter = numIterations_;
	while(mx_iter-- > 0 && rho > goalEps_)
	{
		vDesired = kRho_ * rho;
		gamma = (kAlpha_ * alpha + kBeta_ * beta)/rho;
		steerAngleDesired = atan(gamma);
		robot_->oneMoveStep(vDesired, steerAngleDesired, v, steerAngle, x, y, orient, dt_);
		rho = hypot(x-s2.origX_, y-s2.origY_);
		alpha = minAngle(atan2(s2.origY_-y, s2.origX_-x), orient);
		beta = minAngle(s2.origOrient_, orient);
		gCost += dt_;
		path.emplace_back(x, y, orient, gCost);
	}
}

// Return the time needed to go from one state to another state by simulating the POSQ (steer) function between them.
double Posq::distBySteer(const Map& mp, const State &s1, const State &s2)
{
	double x = s1.x_;
	double y = s1.y_;
	double orient = s1.orient_;
	double v = s1.v_;
	double steerAngle = s1.strAngle_;
	double gCost = s1.gCost_;
	double vDesired, steerAngleDesired, gamma, alpha, beta, rho;
	rho = hypot(x-s2.x_, y-s2.y_);
	alpha = minAngle(atan2(s2.y_-y, s2.x_-x), orient);
	beta = minAngle(s2.orient_, orient);
	int mxIter = numIterations_;
	while(mxIter-- > 0 && rho > eps_)
	{
		vDesired = kRho_ * rho;
		gamma = (kAlpha_ * alpha + kBeta_ * beta)/rho;
		steerAngleDesired = atan(gamma);
		robot_->oneMoveStep(vDesired, steerAngleDesired, v, steerAngle, x, y, orient, dt_);
		rho = hypot(x-s2.x_, y-s2.y_);
		alpha = minAngle(atan2(s2.y_-y, s2.x_-x), orient);
		beta = minAngle(s2.orient_, orient);
		if(mp.cellIsObstacle(x, y)){
			return 2e9;
		}
		gCost += dt_;
	}
	if(rho > eps_){
		gCost = hypot(x-s2.x_, y-s2.y_) + 1e9;
	}
	return gCost;
}

double Posq::simulationMaxDistance(){
	return dt_*numIterations_*(robot_->getMaxV());
}

Posq *Posq::clone() {
    return new Posq(*this);
}

Posq::Posq(const Posq &other) {
    kRho_ = other.kRho_;
    kBeta_ = other.kBeta_;
    kAlpha_ = other.kAlpha_;
    eps_ = other.eps_;
    dt_ = other.dt_;
    numIterations_ = other.numIterations_;
    robot_ = other.robot_->clone();
}

double Posq::getKRho() const {
    return kRho_;
}

double Posq::getKAlpha() const {
    return kAlpha_;
}

double Posq::getKBeta() const {
    return kBeta_;
}

double Posq::getEps() const {
    return eps_;
}

double Posq::getDt() const {
    return dt_;
}

double Posq::getGoalEps() const {
    return goalEps_;
}

int Posq::getNumIterations() const {
    return numIterations_;
}

void Posq::setKRho(double kRho) {
    kRho_ = kRho;
}

void Posq::setKAlpha(double kAlpha) {
    kAlpha_ = kAlpha;
}

void Posq::setKBeta(double kBeta) {
    kBeta_ = kBeta;
}

void Posq::setEps(double eps) {
    eps_ = eps;
}

void Posq::setDt(double dt) {
    dt_ = dt;
}

void Posq::setGoalEps(double goalEps) {
    goalEps_ = goalEps;
}

void Posq::setNumIterations(int numIterations) {
    numIterations_ = numIterations;
}
