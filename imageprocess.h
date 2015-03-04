#pragma once
#include "ardrone\ardrone.h"
class ImageProcess{
private:
	cv::Mat tmp_img;
public:
	cv::VideoCapture cap;
	//�r�f�I�L���v�`�����g��
	//�g����󋵂Ȃ�0,�g���Ȃ��󋵂Ȃ�-1���������Ă���
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
};