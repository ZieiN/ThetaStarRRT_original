#pragma once
#include <map>
#include <vector>
#include "state.h"
#include "posq.h"
#include "map.h"
#include "position.h"
#include <string>
#include <utility>
#include <fstream>
using namespace std;

class Rrt{
public:
	Rrt();
	~Rrt();
	Rrt(const Rrt&);
    Rrt &operator=(const Rrt &other);
    void setParams(map<string, double>&, Posq *posq);
	bool rrtOnPath(const Map &mp, const vector<Position>& geoPath, double initOrient, double goalOrient);
	State anyAngleSampling(const Map &mp, const vector<Position>& geoPath, double l, double w);
	int nearestNeighborSearch(const Map &mp, const State &x);
	bool isThereLineOfSight(const Map &mp, int x0, int y0, int x1, int y1);
    pair<bool, State> steer(const Map &mp, const State &s1, const State &s2, double eps);
    void extractTrajectory(const Map &mp, const vector<State>& tree, vector<State>& path, State lastState);
    double distBySteer(const Map &mp, const State &s1, const State &s2, double eps);
    vector<State> getPath();
	double getWidthSamplingCorridor_();
	double getBiasOrientation();
	double getStepOnTheStraightPath();
	double getWidthOfTheStepOnTheStraightLine();
	double getLenSamplingOnTheCorner();
	double getGoalEps();
	double getGoalOrientEps();
	Posq* getPosq();
	int getNumSamplingNodes();
	void setWidthSamplingCorridor_(double widthSamplingCorridor);
	void setBiasOrientation(double biasOrientation);
	void setStepOnTheStraightPath(double stepOnTheStraightPath);
	void setWidthOfTheStepOnTheStraightLine(double widthOfTheStepOnTheStraightLine);
	void setLenSamplingOnTheCorner(double lenSamplingOnTheCorner);
	void setGoalEps(double goalEps);
	void setGoalOrientEps(double goalOrientEps);
	void setNumSamplingNodes(int numSamplingNodes);
	void setPosq(Posq* posq);
private:
	double widthSamplingCorridor_;
	double biasOrientation_;
	double stepOnTheStraightPath_;
	double widthOfTheStepOnTheStraightLine_;
	double lenSamplingOnTheCorner_;
	double eps_;
	double goalEps_;
	double goalOrientEps_;
	int numSamplingNodes_;
	Posq *posq_;
	vector<State> tree_;
	vector<State> path_;
	ofstream out;
};