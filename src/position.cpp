#include "position.h"


Position::Position(){
	//default constructor
}
Position::~Position(){
	//default destructor
}
Position::Position(int x, int y){
	x_ = x;
	y_ = y;
}
Position::Position(const Position &other){
	x_ = other.x_;
	y_ = other.y_;
}