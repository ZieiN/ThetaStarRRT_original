#pragma once
#include <bits/stdc++.h>
#include "position.h"
#include "node.h"
#include "map.h"
using namespace std;
class ThetaStar{
public:
    ThetaStar();
    ThetaStar(const ThetaStar &other);
    ThetaStar & operator=(const ThetaStar &other);
    ~ThetaStar();
    bool search(const Map &mp, int st_x, int st_y, int nd_x, int nd_y);
    vector<Position> getGeoPath();
private:
    vector<Position> geoPath_;
    const int kDx_[8] = {0, 0, 1, 1, 1, -1, -1, -1};
    const int kDy_[8] = {1, -1, 0, 1, -1, 0, 1, -1};
    const int kNumOfDirections_ = 8;
    vector<Node> closed_;
    set<Node> open_;
    vector<vector<bool>> vis_;
    vector<vector<double>> cost_;
};