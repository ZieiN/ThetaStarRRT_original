#pragma once
class Position{
public:
	Position();
	~Position();
	Position(int, int);
	Position(const Position&);
	Position &operator=(const Position &other);
	int x_, y_;
};