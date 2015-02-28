#include "ardrone/ardrone.h"
#include <iostream>
#include <ctime>

using namespace cv;
using namespace std;

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

	Mat kao;
	
	//double scale = 4.0;
	for (;;)
	{
		cout << cap.get(CV_CAP_PROP_FPS) << endl;
		Mat frame;
		cap >> frame; // �J��������V�����t���[�����擾
		cvtColor(frame, kao, CV_BGR2GRAY);

		// �������ԒZ�k�̂��߂ɉ摜���k��
		//Mat smallkao(cv::saturate_cast<int>(kao.rows / scale), cv::saturate_cast<int>(kao.cols / scale), CV_8UC1);
		//cv::resize(kao, smallkao, smallkao.size(), 0, 0, cv::INTER_LINEAR);
		//cv::equalizeHist(smallkao, smallkao);
		cv::equalizeHist(kao, kao);

		// ���ފ�̓ǂݍ���
		std::string cascadeName = "haarcascade_frontalface_alt.xml"; // Haar-like
		//std::string cascadeName = "./lbpcascade_frontalface.xml"; // LBP
		cv::CascadeClassifier cascade;
		if (!cascade.load(cascadeName))
			return -1;

		std::vector<cv::Rect> faces;
		// �}���`�X�P�[���i��j�T��
		// �摜�C�o�͋�`�C�k���X�P�[���C�Œ��`���C�i�t���O�j�C�ŏ���`
		/*cascade.detectMultiScale(smallkao, faces,
			1.1, 2,
			CV_HAAR_SCALE_IMAGE
			,
			cv::Size(30, 30));*/
		cascade.detectMultiScale(kao, faces,
			1.1, 2,
			CV_HAAR_SCALE_IMAGE
			,
			cv::Size(30, 30));

		// ���ʂ̕`��
		std::vector<cv::Rect>::const_iterator r = faces.begin();
		for (; r != faces.end(); ++r) {
			cv::Point center;
			int radius;
			//center.x = cv::saturate_cast<int>((r->x + r->width*0.5)*scale);
			//center.y = cv::saturate_cast<int>((r->y + r->height*0.5)*scale);
			//radius = cv::saturate_cast<int>((r->width + r->height)*0.25*scale);
			center.x = cv::saturate_cast<int>((r->x + r->width*0.5));
			center.y = cv::saturate_cast<int>((r->y + r->height*0.5));
			radius = cv::saturate_cast<int>((r->width + r->height)*0.25);
			cv::circle(kao, center, radius, cv::Scalar(80, 80, 255), 3, 8, 0);
		}

		cv::namedWindow("result", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
		cv::imshow("result", kao);

		if (waitKey(30) >= 0) break;
	}
	// VideoCapture �f�X�g���N�^�ɂ��C�J�����͎����I�ɏI����������܂�

	return 0;
}