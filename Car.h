#pragma once
#include "imageprocess.h"
class Car{
public:
	//パラメータは２次元の現在位置、速度
	//位置はドローンに対しての相対位置
	//速度は速度ベクトル
	//x,y軸の取り方は今見えている画像の軸方向にする
	double x, y;
	double vx, vy;
	Car();
	void calcPosition(cv::Point2f p);
	void calcVelocity();
	void Move();
};