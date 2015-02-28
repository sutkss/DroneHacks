#include "ardrone/ardrone.h"
#include <ctime>

using namespace cv;


// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------
int main(int argc, char *argv[])
{
	VideoCapture cap(0); // �f�t�H���g�J�������I�[�v��
	cap.set(CV_CAP_PROP_FPS, 30);
	if (!cap.isOpened())  // �����������ǂ������`�F�b�N
		return -1;
	Mat gray_img;
	for (;;)
	{
		Mat frame;
		cap >> frame; // �J��������V�����t���[�����擾
		cvtColor(frame, gray_img, CV_BGR2GRAY);

		// �q�X�g�O�������R��
		cv::equalizeHist(gray_img,gray_img);

		// ������
		cv::GaussianBlur(gray_img,gray_img, cv::Size(11, 11), 10, 10);

		// �~���o
		std::vector<cv::Vec3f> circles;
		cv::HoughCircles(gray_img, circles, CV_HOUGH_GRADIENT, 1, 100, 20, 40);

		cv::Point center;
		int radius;
		int radius_max = 0;
		std::vector<cv::Vec3f>::iterator it = circles.begin();
		for (; it != circles.end(); ++it) {
			radius = cv::saturate_cast<int>((*it)[2]);
			if (radius > radius_max && radius < 1000 && radius > 10){
				center = cv::Point(cv::saturate_cast<int>((*it)[0]), cv::saturate_cast<int>((*it)[1]));
				radius_max = radius;
			}
		}
		cv::circle(frame, center, radius_max, cv::Scalar(0, 0, 255), 2);

		cv::imshow("circle", frame);

		if (waitKey(30) >= 0) break;
	}
	// VideoCapture �f�X�g���N�^�ɂ��C�J�����͎����I�ɏI����������܂�

	return 0;
}