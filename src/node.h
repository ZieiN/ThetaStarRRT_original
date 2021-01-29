#pragma once
class Node
{
public:
    Node();
    ~Node();
    Node(int x, int y, int father, double gCost, double hCost);
    Node (const Node &other);
    Node &operator=(const Node &other);
    friend bool operator < (const Node &, const Node &);
    int x_, y_, father_;
    double gCost_, hCost_;
};