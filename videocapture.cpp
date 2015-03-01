#include "ardrone/ardrone.h"
#include <ctime>

using namespace cv;

#define epsilon 50

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
	Mat edge_img;
	for (;;)
	{
		Mat frame;
		cap >> frame; // �J��������V�����t���[�����擾
		cvtColor(frame, gray_img, CV_BGR2GRAY);
		edge_img = gray_img;

		// �q�X�g�O�������R��
		cv::equalizeHist(gray_img,gray_img);
		edge_img = gray_img;

		// canny�ɂ��edge���o
		Canny(edge_img, edge_img, 50, 200);

		// ������
		cv::GaussianBlur(gray_img,gray_img, cv::Size(11, 11), 4, 4);
		cv::GaussianBlur(edge_img, edge_img, cv::Size(11, 11), 4, 4);
	
		// �~���o
		std::vector<cv::Vec3f> circles;
		std::vector<cv::Vec3f> circles2;
		cv::HoughCircles(gray_img, circles, CV_HOUGH_GRADIENT, 1, 100, 20,65);
		cv::HoughCircles(edge_img, circles2, CV_HOUGH_GRADIENT, 1, 100, 20, 65);
		cv::Point center;
		cv::Point center2;
		cv::Point genuine_center;
		int radius;
		int radius2;
		int genuine_radius = 0;
		double ep = 1000;
		std::vector<cv::Vec3f>::iterator it = circles.begin();
		std::vector<cv::Vec3f>::iterator it2 = circles2.begin();
		
		for (; it != circles.end(); ++it) {
			radius = cv::saturate_cast<int>((*it)[2]);
			center = cv::Point(cv::saturate_cast<int>((*it)[0]), cv::saturate_cast<int>((*it)[1]));
			//cv::circle(frame, center, radius, cv::Scalar(0, 255, 0), 2);
			for (; it2 != circles2.end(); ++it2){
				radius2 = cv::saturate_cast<int>((*it2)[2]);
				center2 = cv::Point(cv::saturate_cast<int>((*it2)[0]), cv::saturate_cast<int>((*it2)[1]));
				//cv::circle(frame, center2, radius2, cv::Scalar(255, 0, 0), 2);

				// �G�b�W�̉摜�ƃ��m�N���̉摜���瓾��ꂽ�~�̌덷
				int delta_r = abs(radius - radius2);
				int delta_x = abs(center.x - center2.x);
				int delta_y = abs(center.y - center2.y);
				if (delta_r < epsilon){
					if (delta_x < epsilon/sqrt(2) && delta_y < epsilon/sqrt(2)){
						// �덷���ł��������~��ۑ�
						if (ep>delta_r+delta_x+delta_y){
							ep = delta_r + delta_x + delta_y;
							genuine_center.x = (center.x + center2.x) / 2;
							genuine_center.y = (center.y + center2.y) / 2;
							genuine_radius = (radius + radius2) / 2;
						}
					}
				}
			}
		}
		// �덷���ł��������~��`��
		cv::circle(frame, genuine_center, genuine_radius, cv::Scalar(0, 0, 255), 2);

		// �`��
		cv::imshow("circle", frame);

		if (waitKey(30) >= 0) break;
	}

	return 0;
}