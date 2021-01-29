#include <bits/stdc++.h>
#include "thetastar.h"
#include "map.h"
using namespace std;

ThetaStar::ThetaStar() {
    geoPath_.clear();
    open_.clear();
    closed_.clear();
    vis_.clear();
    cost_.clear();
}
ThetaStar::~ThetaStar() {
    // default destructor
}

ThetaStar::ThetaStar(const ThetaStar &other){
    closed_ = other.closed_;
    open_ = other.open_;
    vis_ = other.vis_;
    cost_ = other.cost_;
    geoPath_ = other.geoPath_;
}

ThetaStar &ThetaStar::operator=(const ThetaStar &other) {
    if (this != &other)
    {
        closed_ = other.closed_;
        open_ = other.open_;
        vis_ = other.vis_;
        cost_ = other.cost_;
        geoPath_ = other.geoPath_;
    }
    return *this;
}

bool ThetaStar::search(const Map &mp, int st_x, int st_y, int nd_x, int nd_y)
{
    int mpHeight = mp.getHeight();
    int mpWidth = mp.getWidth();
    vis_ = vector<vector<bool>>(mpHeight, vector<bool>(mpWidth, false));
    cost_ = vector<vector<double>>(mpHeight, vector<double>(mpWidth, 0));
    geoPath_.clear();
    open_.clear();
    closed_.clear();
    open_.insert(Node(st_x, st_y, -1, 0, hypot(st_x - nd_x, st_y - nd_y)));
    Node top;
    while(!open_.empty())
    {
        top = *open_.begin();
        open_.erase(open_.begin());
        int x = top.x_;
        int y = top.y_;
        double gCost = top.gCost_;
        int father = top.father_;
        if(x == nd_x && y == nd_y)
        {
            while(x != st_x || y != st_y)
            {
                geoPath_.push_back(Position(x, y));
                x = closed_[father].x_;
                y = closed_[father].y_;
                father = closed_[father].father_;
            }
            geoPath_.push_back(Position(x, y));
            break;

        }
        if(vis_[x][y])
            continue;
        vis_[x][y] = true;
        closed_.push_back(top);
        int current_node = (int)closed_.size() - 1;
        for(int i = 0; i < kNumOfDirections_; ++i)
        {
            int x_tmp = x + kDx_[i];
            int y_tmp = y + kDy_[i];
            if(mp.cellIsObstacle(x_tmp, y_tmp) || mp.cellIsObstacle(x, y_tmp) || mp.cellIsObstacle(x_tmp, y))
                continue;
            if(vis_[x_tmp][y_tmp])
                continue;
            double gCost_tmp = gCost + hypot(x - x_tmp, y - y_tmp);
            double hCost_tmp = hypot(x_tmp - nd_x, y_tmp - nd_y);
            int father_tmp = current_node;
            if(father != -1)
            {
                Node *pnt = &closed_[father];
                if(kDx_[i] * (y - (pnt->y_)) == kDy_[i] * (x - (pnt->x_)))
                {
                    gCost_tmp = pnt->gCost_ + hypot(x_tmp - pnt->x_, y_tmp - pnt->y_);
                    father_tmp = father;
                }
                else if(mp.isThereLineOfSight(pnt->x_, pnt->y_, x_tmp, y_tmp))
                {
                    double c = (pnt->gCost_) + hypot(x_tmp - (pnt->x_), y_tmp - (pnt->y_));
                    father_tmp = father;
                    gCost_tmp = c;
                }
            }
            if(cost_[x_tmp][y_tmp] > gCost_tmp || cost_[x_tmp][y_tmp] == 0)
            {
                if(cost_[x_tmp][y_tmp] > 0)
                {
                    open_.erase(open_.find(Node(x_tmp, y_tmp, father_tmp, cost_[x_tmp][y_tmp], hCost_tmp)));
                }
                open_.insert(Node(x_tmp, y_tmp, father_tmp, gCost_tmp, hCost_tmp));
                cost_[x_tmp][y_tmp] = gCost_tmp;
            }
        }
    }
    if(geoPath_.size() == 0)
        return false;
    reverse(geoPath_.begin(), geoPath_.end());
    return true;
}


vector<Position> ThetaStar::getGeoPath(){
    return geoPath_;
}