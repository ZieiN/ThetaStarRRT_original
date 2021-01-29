#include "readconfigfunctions.h"
#include <string>
#include <vector>
#include <map>
#include <sstream>
#include <fstream>
#include <iostream>
using namespace std;


map<string, double> getRrtParam(){
	ifstream in;
	in.open("../../Config/rrt_params.ini");
	string line, word;
	map<string, double> mp;
	while(getline(in, line)){
		if(line.size()==0){
			continue;
		}
		istringstream iss(line);
		vector<string> v;
		while(iss>>word){
			v.push_back(word);
		}
		if(*v.begin()=="#")
			continue;
		mp[*v.begin()] = stod(*--v.end());
	}
	in.close();
	return mp;
}

map<string, double> getRobotParam(){
	ifstream in;
	in.open("../../Config/robot_params.ini");
	string line, word;
	map<string, double> mp;
	while(getline(in, line)){
		if(line.size()==0){
			continue;
		}
		istringstream iss(line);
		vector<string> v;
		while(iss>>word){
			v.push_back(word);
		}
		if(*v.begin()=="#")
			continue;
		mp[*v.begin()] = stod(*--v.end());
	}
	in.close();
	return mp;
}

map<string, double> getPosqParam(){
	ifstream in;
	in.open("../../Config/posq_params.ini");
	string line, word;
	map<string, double> mp;
	while(getline(in, line)){
		if(line.size()==0){
			continue;
		}
		istringstream iss(line);
		vector<string> v;
		while(iss>>word){
			v.push_back(word);
		}
		if(*v.begin()=="#")
			continue;
		mp[*v.begin()] = stod(*--v.end());
	}
	in.close();
	return mp;
}

string getRobotType(){
	ifstream in;
	in.open("../../Config/robot_type.ini");
	string line, word;
	while(getline(in, line)){
		if(line.size()==0){
			continue;
		}
		istringstream iss(line);
		vector<string> v;
		while(iss>>word){
			v.push_back(word);
		}
		if(*v.begin()=="#")
			continue;
		return *--v.end();
	}
	in.close();
	return "";
}
