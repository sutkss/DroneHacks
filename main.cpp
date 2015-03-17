#include "ardrone/ardrone.h"
#include "ardrone/drone.h"
#include "ardrone\piddrone.h"
#include "imageprocess.h"
#include "Car.h"
#include <ctime>
#include <iostream>
#include <fstream>
#include <list>
using namespace std;

// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------

//DRONE use => 1, not => 0
#define USE_DRONE 1

PIDDrone ardrone;
ImageProcess ImgProc;
Car BrackCircleCar;

//����������
void InitProcess(){
	if(USE_DRONE) ardrone.initialization();
	ImgProc.useVideoCapture();
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
	//debug�p
	ofstream ofs("debug.log");
	std::cerr.rdbuf(ofs.rdbuf());
	std::cerr << "sensor_vx sensor_vy sensor_vz vel.x vel.y"<< std::endl;
	using namespace cv;
	InitProcess();

	cv::Mat prev_img = getImage();
	cv::Mat curr_img = getImage();

	double vx=0.0, vy=0.0, vz=0.0, vr=0.0;
	std::list<cv::Point2f> car_velocity;
	while (1) {
		//���[�v���Ƃ�drone�̉摜���擾
		curr_img = getImage();
		/*�摜�����̕���*/
		ardrone.setParameters(0, 0, 0, 0);
		//�I�v�e�B�J���t���[
		//cv::Mat processed_image = ImgProc.OpticalFlow(prev_img, curr_img);
		cv::Point2f pos = ImgProc.getPosCircleDetection(curr_img);
		if (car_velocity.size() >= 10){
			car_velocity.pop_front();
		}
		//car_velocity.push_back(ImgProc.getVelocityOpticalFlow(prev_img, curr_img));
		//cout << calcCenter(car_velocity) << endl;
		//ImgProc.DrawLine(curr_img, pos, pos + 3*calcCenter(car_velocity));
		cv::Point2f vel = ImgProc.getVelocityOpticalFlow(prev_img, curr_img);
		ardrone.PIDControl(cv::Point2f(pos.y, pos.x));
		//�猟�o
		//cv::Mat processed_image1 = ImgProc.FaceDetection(curr_img);
		//cv::Mat processed_image2 = ImgProc.Labeling(curr_img);
		//cv::Mat processed_image3 = ImgProc.CircleDetection(curr_img);
		//cv::Mat processed_image4 = ImgProc.LineDetection(curr_img);
		//�~���o���Ē��S���W��pos�ɑ��
		/*cv::Point2f pos = ImgProc.getPosCircleDetection(curr_img);
		if (pos != cv::Point2f(-1, -1)){
			//�I�v�e�B�J���t���[����ړ����̂̑��x���v�Z
			cv::Point2f vel = ImgProc.getVelocityOpticalFlow(prev_img, curr_img);
			prev_img = curr_img;
			// ���o���ꂽ�~�̒��S��`��
			
		}
		cv::circle(curr_img, pos, 3, CV_RGB(0, 255, 0), -1, 8, 0);
		ardrone.PIDControl(cv::Point2f(pos.y, pos.x));
		*/
		//BrackCircleCar.calcPosition(ImgProc.getPosCircleDetection(curr_img));
		//BrackCircleCar.calcVelocity(ImgProc.getVelocityOpticalFlow(prev_img, curr_img));
		//ardrone�̑��x�p�����[�^�ύX

		//�h���[���̐��䕔
		//�I�v�e�B�J���t���[�͖������Ȃ̂ő��x��(0,0)��n���Ă�
		//ardrone.brain(pos, cv::Point2f(0,0), curr_img);
		
		
  		//default�̓���������
		if (ardrone.getAvailable()){
			if (ardrone.default_move() == -1)
				break;
		}
		//ardrone�̃p�����[�^�̕����ɓ���
		//cout << ardrone.getvx() << ":" << ardrone.getvy() << ":" << ardrone.getvz() << endl;
		ardrone.Move();
		//double vx, vy;
		//ardrone.getVelocity(&vx, &vy);
		//std::cout << vx << " " << vy << std::endl;
		// Display the image
		cv::imshow("image", curr_img);
		//cv::imshow("processed_image", processed_image);
		//cv::imshow("processed_image1", processed_image1);
		//cv::imshow("processed_image2", processed_image2);
		//cv::imshow("processed_image3", processed_image3);
		//cv::imshow("processed_image4", processed_image4);
		prev_img = curr_img;
		if (!ardrone.getAvailable()){
			if (waitKey(30) >= 0) break;
		}
	}
	return 0;
}