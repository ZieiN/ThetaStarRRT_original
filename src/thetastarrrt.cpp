#include "rrt.h"
#include "thetastar.h"
#include "thetastarrrt.h"
#include "state.h"
#include <iostream>
#include <map>
#include <vector>
using namespace std;


ThetaStarRrt::ThetaStarRrt(){
    // default constructor
}
ThetaStarRrt::~ThetaStarRrt(){
    //default destructor
}

ThetaStarRrt::ThetaStarRrt(const ThetaStarRrt &other){
    path_ = other.path_;
    thetaStar_ = other.thetaStar_;
    rrt_ = other.rrt_;
}

ThetaStarRrt::ThetaStarRrt(ThetaStar thetaStar, Rrt rrt){
    thetaStar_ = thetaStar;
    rrt_ = rrt;
}

bool ThetaStarRrt::search(int stX, int stY, double stOrient, int goalX, int goalY, double goalOrient, const Map &mp){
    const clock_t beginTime = clock();
    cout<<"Theta* started.."<<endl;
    bool geoPathFound = thetaStar_.search(mp, stX, stY, goalX, goalY);
    if(!geoPathFound) {
        return false;
    }
    vector<Position> geoPath = thetaStar_.getGeoPath();
    times1 = double( clock () - beginTime ) /  CLOCKS_PER_SEC;
    cout<<"Theta* finished.. Time elapsed till now: "<<double( clock () - beginTime ) /  CLOCKS_PER_SEC<<endl;
    cout<<"RRT started.."<<endl;
    bool pathFound = rrt_.rrtOnPath(mp, geoPath, stOrient, goalOrient);
    if(!pathFound)
        return false;
    path_ = rrt_.getPath();
    cout <<"RRT finished.. Time elapsed till now: "<< double( clock () - beginTime ) /  CLOCKS_PER_SEC<<endl;
    times=double( clock () - beginTime ) /  CLOCKS_PER_SEC;
    return true;
}
vector<State> ThetaStarRrt::getPath(){
    return path_;
}