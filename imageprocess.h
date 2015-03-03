#pragma once
#include "ardrone\ardrone.h"
class ImageProcess{
private:
	cv::Mat tmp_img;
public:
	cv::VideoCapture cap;
	//ビデオキャプチャを使う
	//使える状況なら0,使えない状況なら-1がかえってくる
	int useVideoCapture();
	cv::Mat getVideoCapture();
	//opticalFlow
	cv::Mat OpticalFlow(cv::Mat prev, cv::Mat curr);
	cv::Mat FaceDetection(cv::Mat image);
};