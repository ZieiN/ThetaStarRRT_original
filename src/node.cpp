#include "node.h"
Node::Node(){
    //default constructor
}
Node::~Node(){
    //default destructor
}
Node::Node(int x, int y, int father, double gCost, double hCost){
    x_ = x;
    y_ = y;
    father_ = father;
    gCost_ = gCost;
    hCost_ = hCost;
}
Node::Node (const Node &other)
{
    x_ = other.x_;
    y_ = other.y_;
    father_ = other.father_;
    gCost_ = other.gCost_;
    hCost_ = other.hCost_;
}
bool operator < (const Node &a, const Node &b)
{
    if (a.gCost_ + a.hCost_ == b.gCost_ + b.hCost_)
    {
        if(a.hCost_ == b.hCost_)
        {
            if(a.x_ == b.x_){
                return a.y_ < b.y_;
            }
            return a.x_ < b.x_;
        }
        return a.hCost_ < b.hCost_;
    }
    return a.gCost_ + a.hCost_ < b.gCost_ + b.hCost_;
}