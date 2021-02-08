#include <map>
#include <vector>
#include "rrt.h"
#include "state.h"
#include <sstream>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <cassert>
#include "thetastar.h"
#include "ddrobot.h"
#include "carlikerobot.h"
#include "thetastarrrt.h"
#include "readconfigfunctions.h"

using namespace std;
int main() {
    ios_base::sync_with_stdio(false);
    ifstream in;
    int mpHeight, mpWidth;
    int stX, stY, goalX, goalY;
    double stOrient, goalOrient;
    double pathlength = 0, tm = 0, tm1 = 0;
    set<pair<int, int>> st;
    in.open("../../Input/image4.txt");
    in >> mpHeight >> mpWidth;
    Map mp(mpWidth, mpHeight);
    int cell;
    for (int i = 0; i < mpHeight; ++i) {
        for (int j = 0; j < mpWidth; ++j) {
            in >> cell;
            mp.setCell(i, j, cell);
        }
    }
    in.close();
    in.open("../../Input/input4.txt");
    int suc = 0;
    ofstream t1, t2, l1, l2;
    t1.open("../../Output/Theta_star_time4.txt");
    t2.open("../../Output/Theta_star_rrt_time4.txt");
    l1.open("../../Output/Theta_star_path_length4.txt");
    l2.open("../../Output/Theta_star_rrt_path_length4.txt");
    while (in >> stX >> stY >> stOrient >> goalX >> goalY >> goalOrient) {
//        swap(stX, stY);
//        swap(goalX, goalY);
//        stOrient = (M_PI / 2.0 - stOrient);
//        goalOrient = (M_PI / 2.0 - goalOrient);
        stOrient += M_PI/2.0;
        goalOrient += M_PI/2.0;
        assert(mp.cellIsObstacle(stX, stY) == 0);
        assert(mp.cellIsObstacle(goalX, goalY) == 0);
        map<string, double> rrtParams = getRrtParam();
        map<string, double> robotParams = getRobotParam();
        map<string, double> posqParams = getPosqParam();
        string robotType = getRobotType();
        AbstractRobot *robot;
        if (robotType == "car_like_robot") {
            robot = new CarLikeRobot();
        } else if (robotType == "Husky") {
            robot = new DDRobot();
        } else {
            cerr << "Robot type is not supported" << endl;
            exit(0);
        }
        robot->setParams(robotParams);
        Posq *posq = new Posq;
        posq->setParams(posqParams, robot);
        Rrt rrt;
        rrt.setParams(rrtParams, posq);
        ThetaStar thetaStar;
        ThetaStarRrt thetaStarRrt(thetaStar, rrt);
        bool pathFound = thetaStarRrt.search(stX, stY, stOrient, goalX, goalY, goalOrient, mp);
        delete posq;
        delete robot;
        suc += pathFound;
        if (!pathFound) {
            cerr << "Path not found!" << endl;
            t1<<0<<endl;
            t2<<0<<endl;
            l1<<0<<endl;
            l2<<0<<endl;
            continue;
//            return 0;
        }
        vector<State> path = thetaStarRrt.getPath();
        t1<<thetaStarRrt.time1<<endl;
        t2<<thetaStarRrt.time2<<endl;
        l1<<thetaStarRrt.len1<<endl;
        l2<<thetaStarRrt.len2<<endl;
    }
    in.close();
    t1.close();
    t2.close();
    l1.close();
    l2.close();
    cout<<suc<<endl;
    return 0;
}