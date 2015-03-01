#include "ardrone.h"
#pragma once
class Drone : public ARDrone{
private:
	//速度パラメータ
	double vx;
	double vy;
	double vz;
	double vr;

public:
	Drone();
	bool initialization();
	int default_move();
	void setParameters(double _vx, double _vy, double _vz, double _vr);
	void Move();
};