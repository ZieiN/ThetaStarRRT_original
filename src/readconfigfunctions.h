#pragma once
#include <map>
#include <string>
using namespace std;

map<string, double> getRrtParam();
map<string, double> getRobotParam();
map<string, double> getPosqParam();
string getRobotType();