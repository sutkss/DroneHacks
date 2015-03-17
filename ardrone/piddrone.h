#pragma once
#include "ardrone.h"
#include "drone.h"
#include <cstring>
#include <sstream>
#include <fstream>

const int num_x = 6;
const int num_z = 3;
const int size_x = num_x*num_x + num_x + 2 * num_x + 2 + num_x + num_x * 2 + num_x*num_x + num_x + num_x + num_x * 2;
// 参照値
const double ref[3] = { 0.0, 0.0, 1.7 };

class PIDDrone : public Drone{
public:
	int piddone = 0;
	int i = 0, j = 0;
	int length;

	const std::string para_x = "parameter_x.csv";
	const std::string para_y = "parameter_y.csv";
	const std::string para_z = "parameter_z.csv";

	cv::Mat Ax;
	cv::Mat Bx;
	cv::Mat Cx;
	cv::Mat Dx;
	cv::Mat Kx;
	cv::Mat Lx;
	cv::Mat Axd;
	cv::Mat Bxd;
	cv::Mat Kxd;
	cv::Mat Lxd;

	cv::Mat Ay;
	cv::Mat By;
	cv::Mat Cy;
	cv::Mat Dy;
	cv::Mat Ky;
	cv::Mat Ly;
	cv::Mat Ayd;
	cv::Mat Byd;
	cv::Mat Kyd;
	cv::Mat Lyd;

	cv::Mat Az;
	cv::Mat Bz;
	cv::Mat Cz;
	cv::Mat Dz;
	cv::Mat Kz;
	cv::Mat Lz;
	cv::Mat Azd;
	cv::Mat Bzd;
	cv::Mat Kzd;
	cv::Mat Lzd;


	std::string str;
	std::stringstream ss;

	// 状態方程式の数値計算用配列の確保（1が一番古いデータ）
	cv::Mat state_est_z1 = cv::Mat::zeros(num_z, 1, CV_64FC1);
	cv::Mat state_est_z2 = cv::Mat::zeros(num_z, 1, CV_64FC1);
	cv::Mat state_est_z3 = cv::Mat::zeros(num_z, 1, CV_64FC1);
	cv::Mat output_z1 = cv::Mat::zeros(2, 1, CV_64FC1);
	cv::Mat output_z2 = cv::Mat::zeros(2, 1, CV_64FC1);
	cv::Mat output_z3 = cv::Mat::zeros(2, 1, CV_64FC1);
	double input_z1 = 0;
	double input_z2 = 0;
	double input_z3 = 0;

	cv::Mat state_est_x1 = cv::Mat::zeros(num_x, 1, CV_64FC1);
	cv::Mat state_est_x2 = cv::Mat::zeros(num_x, 1, CV_64FC1);
	cv::Mat state_est_x3 = cv::Mat::zeros(num_x, 1, CV_64FC1);
	cv::Mat output_x1 = cv::Mat::zeros(2, 1, CV_64FC1);
	cv::Mat output_x2 = cv::Mat::zeros(2, 1, CV_64FC1);
	cv::Mat output_x3 = cv::Mat::zeros(2, 1, CV_64FC1);
	double input_x1 = 0;
	double input_x2 = 0;
	double input_x3 = 0;

	cv::Mat state_est_y1 = cv::Mat::zeros(num_x, 1, CV_64FC1);
	cv::Mat state_est_y2 = cv::Mat::zeros(num_x, 1, CV_64FC1);
	cv::Mat state_est_y3 = cv::Mat::zeros(num_x, 1, CV_64FC1);
	cv::Mat output_y1 = cv::Mat::zeros(2, 1, CV_64FC1);
	cv::Mat output_y2 = cv::Mat::zeros(2, 1, CV_64FC1);
	cv::Mat output_y3 = cv::Mat::zeros(2, 1, CV_64FC1);
	double input_y1 = 0;
	double input_y2 = 0;
	double input_y3 = 0;

	cv::Mat k1 = cv::Mat::zeros(num_x, 1, CV_64FC1);
	cv::Mat k2 = cv::Mat::zeros(num_x, 1, CV_64FC1);
	cv::Mat k3 = cv::Mat::zeros(num_x, 1, CV_64FC1);
	cv::Mat k4 = cv::Mat::zeros(num_x, 1, CV_64FC1);

	cv::Mat k1z = cv::Mat::zeros(num_z, 1, CV_64FC1);
	cv::Mat k2z = cv::Mat::zeros(num_z, 1, CV_64FC1);
	cv::Mat k3z = cv::Mat::zeros(num_z, 1, CV_64FC1);
	cv::Mat k4z = cv::Mat::zeros(num_z, 1, CV_64FC1);

	double input_yaw = 0;

	// 円の中心(原点)座標
	float x1 = 0, y1 = 0;
	// 視野角
	double alpha = 0.44;
	double beta = 0.257;
	// 機体中心からカメラまでの距離
	double d = 0.05;
	double z = 0;
	double h;

	// Time
	long start;
	double last;

	// 前の速度
	cv::Mat V_last = cv::Mat::zeros(3, 1, CV_64FC1);

	// 内部センサ(加速度センサ)から求まる機体の位置（絶対座標系）
	cv::Mat _P = cv::Mat::zeros(3, 1, CV_64FC1);

	// 内部センサから求まる機体の速度（絶対座標系）
	cv::Mat V = cv::Mat::zeros(3, 1, CV_64FC1);

	// yaw_start
	double yaw_start = getYaw();

	SYSTEMTIME st;

	double Mode = 0;
	PIDDrone();
	
	virtual void move3D2(double* vx, double* vy, double* vz, double* vr);
	void PIDControl(cv::Point2f pos);
};