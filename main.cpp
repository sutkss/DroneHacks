#include "ardrone/ardrone.h"
#include "ardrone/drone.h"
#include "imageprocess.h"
#include "Car.h"
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
Car BrackCircleCar;

//初期化処理
void InitProcess(){
	if(USE_DRONE) ardrone.initialization();
	ImgProc.useVideoCapture();
}

//ドローンと接続されていたらドローンから、
//ビデオキャプチャが取得できればキャプチャを返す
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

	double vx=0.0, vy=0.0, vz=0.0, vr=0.0;
	while (1) {
		//ループごとにdroneの画像を取得
		curr_img = getImage();

		/*画像処理の部分*/

		//オプティカルフロー
		cv::Mat processed_image = ImgProc.OpticalFlow(prev_img, curr_img);
		//顔検出
		//cv::Mat processed_image1 = ImgProc.FaceDetection(curr_img);
		//cv::Mat processed_image2 = ImgProc.Labeling(curr_img);
		//cv::Mat processed_image3 = ImgProc.CircleDetection(curr_img);
		//cv::Mat processed_image4 = ImgProc.LineDetection(curr_img);

		/*制御部分*/
		/*
			.......
		*/
		BrackCircleCar.calcPosition(ImgProc.getPosCircleDetection(curr_img));

		//ardroneの速度パラメータ変更
		vx = -BrackCircleCar.x / (2*(BrackCircleCar.x + BrackCircleCar.y));
		vy = -BrackCircleCar.y / (2*(BrackCircleCar.x + BrackCircleCar.y));
		ardrone.setParameters(vx, vy, vz, vr);
		

		//defaultの動きをする
		if (ardrone.getAvailable()){
			if (ardrone.default_move() == -1)
				break;
		}
		//ardroneのパラメータの方向に動く
		//ardrone.Move();

		// Display the image
		cv::imshow("processed_image", processed_image);
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