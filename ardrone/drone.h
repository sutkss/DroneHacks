#include "ardrone.h"
#pragma once
class Drone : public ARDrone{
private:
	//速度パラメータ
	double vx;
	double vy;
	double vz;
	double vr;
	//Droneのリソースが利用可能かどうか
	bool available;
public:
	/*パラメータ初期化*/
	Drone();
	/*ドローンに接続とかの初期化処理*/
	bool initialization();
	/*デフォルトであったキー操作の動きの処理*/
	int default_move();
	/*速度パラメータの設定処理*/
	void setParameters(double _vx, double _vy, double _vz, double _vr);
	/*速度パラメータの取得処理*/
	double getvx(){ return vx; }
	double getvy(){ return vy; }
	double getvz(){ return vz; }
	double getvr(){ return vr; }
	void setvx(double _vx){ vx = _vx; }
	void setvy(double _vy){ vy = _vy; }
	void setvz(double _vz){ vz = _vz; }
	void setvr(double _vr){ vr = _vr; }
	/*パラメータの方向に動く*/
	virtual void Move();
	void brain(cv::Point2f pos, cv::Point2f v, cv::Mat img);
	bool getAvailable(){
		return available;
	}
	virtual ~Drone(){
		close();
	}
};