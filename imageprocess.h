#pragma once
#include "ardrone\ardrone.h"
#include <list>

float vector_distance(cv::Point2f p1, cv::Point2f p2);
cv::Point2f calcCenter(std::list<cv::Point2f> data);
void Kmeans2(std::vector<cv::Point2f> data, std::vector<cv::Point2f> points, std::list<cv::Point2f> *ret);

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
	cv::Mat OpticalFlow(cv::Mat _prev, cv::Mat _curr);
	cv::Mat FaceDetection(cv::Mat _image);
	cv::Mat Labeling(cv::Mat _image);
	cv::Mat CircleDetection(cv::Mat _image);
	cv::Mat LineDetection(cv::Mat _image);
	cv::Point2f getPosCircleDetection(cv::Mat _image);
	cv::Point2f getVelocityOpticalFlow(cv::Mat _prev, cv::Mat _curr);
	void ImageProcess::DrawLine(cv::Mat &image, cv::Point2f from, cv::Point2f to);
};