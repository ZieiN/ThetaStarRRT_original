#include "rrt.h"
#include "thetastar.h"
#include "state.h"

class ThetaStarRrt{
public:
    ThetaStarRrt();
    ~ThetaStarRrt();
    ThetaStarRrt(const ThetaStarRrt &other);
    ThetaStarRrt &operator = (const ThetaStarRrt &other);
    ThetaStarRrt(ThetaStar, Rrt);
    bool search(int stX, int stY, double stOrient, int goalX, int goalY, double goalOrient, const Map &mp);
    vector<State> getPath();
    double time1, time2, len1, len2;
private:
    ThetaStar thetaStar_;
    Rrt rrt_;
    vector<State> path_;
};