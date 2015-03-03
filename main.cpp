#include "ardrone/ardrone.h"
#include "ardrone/drone.h"
#include "imageprocess.h"
#include <ctime>
#include <iostream>
using namespace std;

// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------

//DRONE use => 1, not => 0
#define USE_DRONE 0

Drone ardrone;
ImageProcess ImgProc;

//����������
void InitProcess(){
	if(USE_DRONE) ardrone.initialization();
	else ImgProc.useVideoCapture();
}

//�h���[���Ɛڑ�����Ă�����h���[������A
//�r�f�I�L���v�`�����擾�ł���΃L���v�`����Ԃ�
cv::Mat getImage(){
	if (ardrone.getAvailable()){
		return ardrone.getImage();
	}
	else if(ImgProc.cap.isOpened()){
		return ImgProc.getVideoCapture();
	}
	else{
		return cv::Mat();
	}
}

int main(int argc, char *argv[])
{
	using namespace cv;
	InitProcess();

	cv::Mat prev_img = getImage();
	cv::Mat curr_img = getImage();
	double vx, vy, vz, vr;
	while (1) {
		//���[�v���Ƃ�drone�̉摜���擾
		curr_img = getImage();

		/*�摜�����̕���*/
		/*
			.......
		*/
		//�I�v�e�B�J���t���[
		//cv::Mat processed_image = ImgProc.OpticalFlow(prev_img, curr_img);
		//�猟�o
		//cv::Mat processed_image = ImgProc.FaceDetection(curr_img);
		//cv::Mat processed_image = ImgProc.Labeling(curr_img);
		//cv::Mat processed_image = ImgProc.CircleDetection(curr_img);
		cv::Mat processed_image = ImgProc.LineDetection(curr_img);
		/*���䕔��*/
		/*
			.......
		*/

		//ardrone�̑��x�p�����[�^�ύX
		/*
		ardrone.setParameters(vx, vy, vz, vr);
		*/

		//default�̓���������
		if (ardrone.getAvailable()){
			if (ardrone.default_move() == -1)
				break;
		}
		//ardrone�̃p�����[�^�̕����ɓ���
		ardrone.Move();

		// Display the image
		cv::imshow("processed_image", processed_image);
		prev_img = curr_img;
		if (!ardrone.getAvailable()){
			if (waitKey(30) >= 0) break;
		}
	}

	return 0;
}