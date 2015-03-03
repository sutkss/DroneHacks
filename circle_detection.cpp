#include "ardrone/ardrone.h"
#include <ctime>

using namespace cv;

#define epsilon 0.1

// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------

double min(double a, double b){
	if (a < b){
		return a;
	}
	return b;
}

int main(int argc, char *argv[])
{
	VideoCapture cap(0); // �f�t�H���g�J�������I�[�v��
	cap.set(CV_CAP_PROP_FPS, 30);
	if (!cap.isOpened())  // �����������ǂ������`�F�b�N
		return -1;

	for (;;)
	{
		// ���̉摜
		Mat frame;
		// �~���o�p�摜
		Mat gray_img;
		// �~���o���r�p�摜
		Mat tmp_img;
		Mat cmp_img;

		double x, y;// �~�̒��S���W

		//Mat frame2;	// test�p
		cap >> frame; // �J��������V�����t���[�����擾
		cvtColor(frame, gray_img, CV_BGR2GRAY);
		//cvtColor(frame, tmp_img, CV_BGR2GRAY);
		cvtColor(frame, cmp_img, CV_BGR2GRAY);
		//cvtColor(frame, frame2, CV_BGR2GRAY);
		// �G�b�W���o
		Canny(cmp_img, cmp_img, 50, 200);
		cmp_img = ~cmp_img;

		// �q�X�g�O�������R��
		cv::equalizeHist(gray_img,gray_img);

		// ������
		cv::GaussianBlur(gray_img,gray_img, cv::Size(11, 11), 11, 11);

		// �~���o
		std::vector<cv::Vec3f> circles;
		cv::HoughCircles(gray_img, circles, CV_HOUGH_GRADIENT, 1, 100, 100, 50);
		cv::Point center;
		cv::Point genuine_center;
		int radius;
		int genuine_radius = 0;
		std::vector<cv::Vec3f>::iterator it = circles.begin();
		double ep=100.0;
		for (; it != circles.end(); ++it) {
			radius = cv::saturate_cast<int>((*it)[2]);
			center = cv::Point(cv::saturate_cast<int>((*it)[0]), cv::saturate_cast<int>((*it)[1]));

			// ���o�����~��`��
			//cv::circle(frame2, center, radius, cv::Scalar(255, 0, 0), 2);
			
			// �~���o��ɔ�r
			cvtColor(frame, tmp_img, CV_BGR2GRAY);
			cv::circle(tmp_img, center, radius, cv::Scalar(0, 0, 0), 2);
				// �G�b�W���o
			Canny(tmp_img, tmp_img, 50, 200);
			tmp_img = ~tmp_img;
			if (cv::matchShapes(tmp_img, cmp_img, CV_CONTOURS_MATCH_I2, 0) < ep ){
				genuine_center = center;
				genuine_radius = radius;
				ep = cv::matchShapes(tmp_img, cmp_img, CV_CONTOURS_MATCH_I2, 0);
			}
		}
		
		// �~�`��
		cv::circle(frame, genuine_center, genuine_radius, cv::Scalar(0, 0, 255), 2);
		// �~���S�`��
		cv::circle(frame, genuine_center, 0, cv::Scalar(0, 255, 0), 2);
		
		x = genuine_center.x;
		y = genuine_center.y;

		cv::imshow("circle", frame);
		//cv::imshow("frame2", frame2);
		if (waitKey(30) >= 0) break;
	}

	return 0;
}