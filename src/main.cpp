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
char c[111];
int l[601][601];
int main()
{
	ios_base::sync_with_stdio(false);
	ifstream in;
	int mpHeight, mpWidth;
	int stX, stY, goalX, goalY;
	double stOrient, goalOrient;
	int cnt = 0, suc[11]={0};
	double pathlength=0, tm=0, tm1=0;
	set<pair<int, int>> st;
	for(int i=0; i<100; ++i) {
	    string s = "../../Input/map_"+to_string(i)+".txt";
	    strcpy(c, s.c_str());
        in.open(c);
        in >> mpHeight >> mpWidth;
        Map mp(mpWidth, mpHeight);
        int cell;
        for(int i = 0; i < mpHeight; ++i)
        {
            for(int j = 0; j < mpWidth; ++j)
            {
                in >> cell;
                l[i][j]=(l[i][j]||cell);
                mp.setCell(i, j, cell);
            }
        }
        in.close();
        cout<<i<<":::"<<endl;
	    cnt = 0;
        in.open("../../Input/inputs.txt");
        while (in >> stX >> stY >> stOrient >> goalX >> goalY >> goalOrient) {
            swap(stX, stY);
            swap(goalX, goalY);
            stOrient = (M_PI/2.0 - stOrient);
            goalOrient = (M_PI/2.0 - goalOrient);
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
//            cout << robotType << endl;
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
//        break;
            suc[cnt++]+=pathFound;
            if (!pathFound) {
                cerr << "Path not found!" << endl;
                continue;
//            return 0;
            }
            vector<State> path = thetaStarRrt.getPath();
            tm1+=thetaStarRrt.times1;
            tm+=thetaStarRrt.times;
//            ofstream Len;
//            Len.open("../Output/path.txt");
//            Len << "Path points with orientation:" << endl;
//            Len << setw(9) << "X" << setw(9) << "Y" << setw(14) << "orientation" << endl;
//            Len << fixed << setprecision(3);
//            cout << "Length of whole path in seconds = " << path.size() * posqParams["dt"] << "s" << endl;
//            for (auto it : path) {
//                Len << setw(9) << it.x_ << setw(9) << it.y_ << setw(14) << it.orient_ << endl;
//            }
double tmp = 0;
for(int i=0; i+1<path.size(); ++i){
    tmp+=hypot(path[i].x_-path[i+1].x_, path[i].y_-path[i+1].y_);
}
//            Len.close();
//        break;
            pathlength+=tmp;
        }
        in.close();break;
    }
//	for(int i=0; i<600;++i){
//	    for(int j=0; j<600; ++j){
//	        if(l[i][j]==0){
//	            cout<<i<<","<<j<<endl;
//	        }
//	    }
//	}
	double count =0;
	for(int i=0; i<cnt; ++i){
	    cout<<suc[i]<<" ";
	    count+=suc[i];
	}
	cout<<pathlength/count<<endl;
	cout<<tm1/count<<endl;
	cout<<tm/count<<endl;
	cout<<endl;
	return 0;
}