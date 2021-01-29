#pragma once
class Position{
public:
	Position();
	~Position();
	Position(int, int);
	Position(const Position&);
	int x_, y_;
};