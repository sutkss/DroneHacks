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
	cap.set(CV_CAP_PROP_FPS, 1);
	if (!cap.isOpened())  // �����������ǂ������`�F�b�N
		return -1;

	for (;;)
	{
		Mat frame;
		cap >> frame; // �J��������V�����t���[�����擾

		cv::namedWindow("result", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
		cv::imshow("result", frame);

		if (waitKey(30) >= 0) break;
	}
	// VideoCapture �f�X�g���N�^�ɂ��C�J�����͎����I�ɏI����������܂�

	return 0;
}