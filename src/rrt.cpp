#include "geometricfunctions.h"
#include "readconfigfunctions.h"
#include "state.h"
#include "robotstate.h"
#include "posq.h"
#include "rrt.h"
#include "map.h"
#include "position.h"
#include <cmath>
#include <string>
#include <map>
#include <vector>
#include <sstream>
#include <fstream>
#include <iostream>
#include <utility>
#include <algorithm>

using namespace std;

Rrt::~Rrt(){
    if(posq_)
        delete posq_;
}
Rrt::Rrt(){
	// default constructor
}

Rrt::Rrt(const Rrt & other) {
widthSamplingCorridor_ = other.widthSamplingCorridor_;
biasOrientation_ = other.biasOrientation_;
stepOnTheStraightPath_ = other.stepOnTheStraightPath_;
widthOfTheStepOnTheStraightLine_ = other.widthOfTheStepOnTheStraightLine_;
eps_ = other.eps_;
goalEps_ = other.goalEps_;
lenSamplingOnTheCorner_ = other.lenSamplingOnTheCorner_;
goalOrientEps_ = other.goalOrientEps_;
numSamplingNodes_ = other.numSamplingNodes_;
posq_ = new Posq(*other.posq_);
}

Rrt &Rrt::operator=(const Rrt &other) {
    if (this != &other)
    {
        widthSamplingCorridor_ = other.widthSamplingCorridor_;
        biasOrientation_ = other.biasOrientation_;
        stepOnTheStraightPath_ = other.stepOnTheStraightPath_;
        widthOfTheStepOnTheStraightLine_ = other.widthOfTheStepOnTheStraightLine_;
        eps_ = other.eps_;
        goalEps_ = other.goalEps_;
        lenSamplingOnTheCorner_ = other.lenSamplingOnTheCorner_;
        goalOrientEps_ = other.goalOrientEps_;
        numSamplingNodes_ = other.numSamplingNodes_;
        posq_ = new Posq(*other.posq_);
    }
    return *this;
}
void Rrt::setParams(map<string, double> &param, Posq *posq){
	widthSamplingCorridor_ = param["width_sampling_corridor"];
	biasOrientation_ = param["bias_orientation"];
	stepOnTheStraightPath_ = param["step_on_the_straight_path"];
	widthOfTheStepOnTheStraightLine_ = param["width_of_the_step_on_the_straight_line"];
	lenSamplingOnTheCorner_ = param["len_sampling_on_the_corner"];
	eps_ = param["EPS"];
	goalEps_ = param["goal_EPS"];
	goalOrientEps_ = param["goal_orient_EPS"];
	numSamplingNodes_ = param["num_sampling_nodes"];
	posq_ = new Posq(*posq);
}
// This function returns true if there are no obstacles in the straight line between two points in the given map.
bool Rrt::isThereLineOfSight(const Map &mp, int x0, int y0, int x1, int y1){
//	if(max(fabs(x0-x1), fabs(y0-y1)) > posq_->simulationMaxDistance()){
//		return false;
//	}
	return mp.isThereLineOfSight(x0, y0, x1, y1);
}

// This function is our implementation of the function "anyAngleSampling" presented in pseudo-code of the algorithm theta*-rrt in
// the original article.
// Given random point along theta* path (we define the point by its projection on the path 'l' and the distance from the path 'w'), 
// this function add random orientation of the point (limited by 'biasOrientation' around the original orientation of the
// segment of the theta* path which is near this point) and convert the point to state in state-space.
State Rrt::anyAngleSampling(const Map &mp, const vector<Position>& geoPath, double l, double w)
{
	double tmp = 0;
	State ans;
	for(int i = 0; i+1 < geoPath.size(); ++i)
	{
		auto cur = geoPath[i];
		auto nxt = geoPath[i + 1];
		if(l >= tmp && l <= tmp + hypot(cur.x_ - nxt.x_, cur.y_ - nxt.y_))
		{
			l -= tmp;
			double ang1 = atan2(nxt.y_ - cur.y_, nxt.x_ - cur.x_);
			double ang2 = ang1 + atan(w / l);
			makeAngleBtwPiMinusPi(ang1);
			makeAngleBtwPiMinusPi(ang2);
			auto hypotenuse = hypot(l, w);
			ans.x_ = cur.x_ + cos(ang2) * hypotenuse;
			ans.y_ = cur.y_ + sin(ang2) * hypotenuse;
			if(i>0){
				double ang = angle(geoPath[i-1].x_, geoPath[i-1].y_, geoPath[i].x_, geoPath[i].y_, ans.x_, ans.y_);
				if(ang<M_PI/2.0 && hypot(ans.x_-geoPath[i].x_, ans.y_-geoPath[i].y_)*sin(ang)<widthSamplingCorridor_/2.0){
					ans.x_-=2*(ans.x_-geoPath[i].x_);
					ans.y_-=2*(ans.y_-geoPath[i].y_);
					double ang3 = atan2(cur.y_-geoPath[i-1].y_, cur.x_-geoPath[i-1].x_);
					if(ang1<ang3)ang1+=2*M_PI;
					if(fabs(ang1-ang3)<2*M_PI-fabs(ang1-ang3))
					    ang1 = (ang1+ang3)/2;
					else
					    ang1 = (ang1+ang3)/2+M_PI;
				}
			}
			double rnd = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
			rnd *= biasOrientation_;
			rnd -= biasOrientation_ / 2.0;
			ans.orient_ = rnd + ang1;
			makeAngleBtwPiMinusPi(ans.orient_);
			ans.x_ = max(ans.x_, (double)0);
			ans.y_ = max(ans.y_, (double)0);
			ans.x_ = min(ans.x_, (double)mp.getHeight()-1);
			ans.y_ = min(ans.y_, (double)mp.getWidth()-1);
			return ans;
		}
		tmp += hypot(cur.x_ - nxt.x_, cur.y_ - nxt.y_);
	}
}

pair<bool, State> Rrt::steer(const Map &mp, const State &s1, const State &s2, double eps) {
    RobotState rs1 = RobotState(s1.x_, s1.y_, s1.orient_, s1.v_, s1.angV_, s1.strAngle_);
    RobotState rs2 = RobotState(s2.x_, s2.y_, s2.orient_, s2.v_, s2.angV_, s2.strAngle_);
    bool pathFound = posq_->steer(rs1, rs2, mp, eps);
    State ans;
    if(!pathFound)
        return {0, ans};
    RobotState lastState = posq_->getLastRobotSate();
    if(hypot(lastState.getX()-s2.x_, lastState.getY()-s2.y_)>eps) {
        return {0, ans};
    }
    double cost = posq_->getPathCost();
    ans = State(lastState.getX(), lastState.getY(), lastState.getOrientation(), s1.gCost_+cost, s2.x_, s2.y_, s2.orient_,
                lastState.getVel(), lastState.getAngVel(), lastState.getSteerAngle(), -1);
    auto v = posq_->getFoundTrajectory();
    for(auto it:v){
        out<<it.getX()<<" "<<it.getY()<<endl;
    }
    return {1, ans};
}


void Rrt::extractTrajectory(const Map &mp, const vector<State>& tree, vector<State>& path, State lastState){
    vector<State> tmp;
    tmp.push_back(lastState);
    while(lastState.parent_ != -1)
    {
        lastState = tree[lastState.parent_];
        tmp.push_back(lastState);
    }
    reverse(tmp.begin(), tmp.end());
    pair<bool, State> pr;
    for(int i = 1; i+1 < tmp.size(); ++i)
    {
        RobotState rs1 = RobotState(tmp[i-1].x_, tmp[i-1].y_, tmp[i-1].orient_, tmp[i-1].v_, tmp[i-1].angV_, tmp[i-1].strAngle_);
        RobotState rs2 = RobotState(tmp[i].origX_, tmp[i].origY_, tmp[i].origOrient_, tmp[i].v_, tmp[i].angV_, tmp[i].strAngle_);
        posq_->steer(rs1, rs2, mp, eps_);
        for(auto it:posq_->getFoundTrajectory())
            path.emplace_back(it.getX(), it.getY(), it.getOrientation());
    }
    RobotState rs1 = RobotState(tmp[(int)tmp.size()-2].x_, tmp[(int)tmp.size()-2].y_, tmp[(int)tmp.size()-2].orient_, tmp[(int)tmp.size()-2].v_, tmp[(int)tmp.size()-2].angV_, tmp[(int)tmp.size()-2].strAngle_);
    RobotState rs2 = RobotState(tmp.back().origX_, tmp.back().origY_, tmp.back().origOrient_, tmp.back().v_, tmp.back().angV_, tmp.back().strAngle_);
    posq_->steer(rs1, rs2, mp, goalEps_);
    for(auto it:posq_->getFoundTrajectory())
        path.emplace_back(it.getX(), it.getY(), it.getOrientation());
}

double Rrt::distBySteer(const Map &mp, const State &s1, const State &s2, double eps) {
    RobotState rs1 = RobotState(s1.x_, s1.y_, s1.orient_, s1.v_, s1.angV_, s1.strAngle_);
    RobotState rs2 = RobotState(s2.x_, s2.y_, s2.orient_, s2.v_, s2.angV_, s2.strAngle_);
    bool pathFound = posq_->steer(rs1, rs2, mp, eps);
    State ans;
    if(!pathFound)
        return 2e9;
    RobotState lastState = posq_->getLastRobotSate();
    if(hypot(lastState.getX()-s2.x_, lastState.getY()-s2.y_) > eps)
        return hypot(lastState.getX()-s2.x_, lastState.getY()-s2.y_) + 1e9;
    return posq_->getPathCost() + s1.gCost_;
}

// In this function, we choose the nearest node according to steer_distance. This function is accurate but taking longer time.
int Rrt::nearestNeighborSearch(const Map &mp, const State &x)
{
	int mnId = -1, mnId2 = 0;
	double mn1 = 2e9, mn2 = dist(tree_[0], x);
	const double deltaSigma = 40;
	for(int i = 0; i < tree_.size(); ++i)
	{
		if(!isThereLineOfSight(mp, x.x_, x.y_, tree_[i].x_, tree_[i].y_))
			continue;
		double d = dist(tree_[i], x);
		if(d < mn2){
		    mn2 = d;
		    mnId2 = i;
		}
		if(d > deltaSigma)
		    continue;
		d = distBySteer(mp, tree_[i], x, eps_);
		if(d < mn1)
		{
			mn1 = d;
			mnId = i;
		}
	}
	if(mnId == -1)
	    return mnId2;
	return mnId;
}

// The main rrt algorithm function.
// Our implementation of sampling method on theta*-path.
bool Rrt::rrtOnPath(const Map &mp, const vector<Position>& geoPath, double initOrient, double goalOrient)
{
	tree_.clear();
	makeAngleBtwPiMinusPi(initOrient);
	makeAngleBtwPiMinusPi(goalOrient);
	tree_.emplace_back(geoPath[0].x_, geoPath[0].y_, initOrient, 0, -1, 0, 0, 0);
	State goal = State(geoPath.back().x_, geoPath.back().y_, goalOrient);
	double mn;
	int mnId = -1;
	double full_len = 0;
	for(int i=0; i+1<geoPath.size(); ++i){
		full_len+=hypot(geoPath[i].x_-geoPath[i+1].x_, geoPath[i].y_-geoPath[i+1].y_);
	}
	out.open("out.txt");
	for(int i = 0; i < numSamplingNodes_; ++i)
	{
		double rnd1 = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
		double rnd2 = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
		rnd1 *= full_len;
		rnd2 *= widthSamplingCorridor_;
		rnd2 -= widthSamplingCorridor_ / 2.0;
		State xRand = anyAngleSampling(mp, geoPath, rnd1, rnd2);
		int xNearId = nearestNeighborSearch(mp, xRand);
        out<<xRand.x_<<" "<<xRand.y_<<" "<<tree_[xNearId].x_<<" "<<tree_[xNearId].y_<<endl;
		if(!isThereLineOfSight(mp, xRand.x_, xRand.y_, tree_[xNearId].x_, tree_[xNearId].y_)){
			continue;
		}
		pair<bool, State> xNew = steer(mp, tree_[xNearId], xRand, eps_);
        out<<xNew.second.x_<<" "<<xNew.second.y_<<endl;
        if(!xNew.first)
		{
			continue;
		}
		xNew.second.parent_ = xNearId;
		tree_.push_back(xNew.second);
		if(dist(xNew.second, goal) <= posq_->simulationMaxDistance()){
			auto toGoalNode = steer(mp, xNew.second, goal, goalEps_);
            out<<toGoalNode.second.x_<<" "<<toGoalNode.second.y_<<" "<<goal.x_<<" "<<goal.y_<<endl;
			if(toGoalNode.first && dist(toGoalNode.second, goal) < goalEps_ && distOrient(toGoalNode.second.orient_, goal.orient_) < goalOrientEps_)
			{
				toGoalNode.second.parent_ = (int)(tree_.size()) - 1;
				toGoalNode.second.origX_ = goal.x_;
				toGoalNode.second.origY_ = goal.y_;
				toGoalNode.second.origOrient_ = goal.orient_;
				mnId = tree_.size();
				tree_.push_back(toGoalNode.second);
				break;
			}
		}
	}
	out.close();
	if(mnId == -1)
		return false;
	extractTrajectory(mp, tree_, path_, tree_[mnId]);
	return true;	
}

vector<State> Rrt::getPath(){
	return path_;
}

double Rrt::getWidthSamplingCorridor_(){
	return widthSamplingCorridor_;
}
double Rrt::getBiasOrientation(){
	return biasOrientation_;
}
double Rrt::getStepOnTheStraightPath(){
	return stepOnTheStraightPath_;
}
double Rrt::getWidthOfTheStepOnTheStraightLine(){
	return widthOfTheStepOnTheStraightLine_;
}
double Rrt::getLenSamplingOnTheCorner(){
	return lenSamplingOnTheCorner_;
}
double Rrt::getGoalEps(){
	return goalEps_;
}
double Rrt::getGoalOrientEps(){
	return goalOrientEps_;
}
int Rrt::getNumSamplingNodes(){
	return numSamplingNodes_;
}
Posq* Rrt::getPosq(){
	return posq_;
}
void Rrt::setWidthSamplingCorridor_(double widthSamplingCorridor){
	widthSamplingCorridor_ = widthSamplingCorridor;
}
void Rrt::setBiasOrientation(double biasOrientation){
	biasOrientation_ = biasOrientation;
}
void Rrt::setStepOnTheStraightPath(double stepOnTheStraightPath){
	stepOnTheStraightPath_ = stepOnTheStraightPath;
}
void Rrt::setWidthOfTheStepOnTheStraightLine(double widthOfTheStepOnTheStraightLine){
	widthOfTheStepOnTheStraightLine_ = widthOfTheStepOnTheStraightLine;
}
void Rrt::setLenSamplingOnTheCorner(double lenSamplingOnTheCorner){
	lenSamplingOnTheCorner_ = lenSamplingOnTheCorner;
}
void Rrt::setGoalEps(double goalEps){
	goalEps_ = goalEps;
}
void Rrt::setGoalOrientEps(double goalOrientEps){
	goalOrientEps_ = goalOrientEps;
}
void Rrt::setNumSamplingNodes(int numSamplingNodes){
	numSamplingNodes_ = numSamplingNodes;
}
void Rrt::setPosq(Posq *posq){
	posq_ = posq;
}
