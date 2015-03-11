#include "ardrone/ardrone.h"
#include "ardrone/drone.h"
#include "imageprocess.h"
#include "Car.h"
#include <ctime>
#include <iostream>
#include <fstream>
using namespace std;

// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------

//DRONE use => 1, not => 0
#define USE_DRONE 1

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
	//debug用
	ofstream ofs("debug.log");
	std::cerr.rdbuf(ofs.rdbuf());

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
		//cv::Mat processed_image = ImgProc.OpticalFlow(prev_img, curr_img);
		//顔検出
		//cv::Mat processed_image1 = ImgProc.FaceDetection(curr_img);
		//cv::Mat processed_image2 = ImgProc.Labeling(curr_img);
		//cv::Mat processed_image3 = ImgProc.CircleDetection(curr_img);
		//cv::Mat processed_image4 = ImgProc.LineDetection(curr_img);


		//円検出して中心座標をposに代入
		cv::Point2f pos = ImgProc.getPosCircleDetection(curr_img);
		//オプティカルフローから移動物体の速度を計算
		cv::Point2f vel = ImgProc.getVelocityOpticalFlow(prev_img, curr_img);

		//BrackCircleCar.calcPosition(ImgProc.getPosCircleDetection(curr_img));
		//BrackCircleCar.calcVelocity(ImgProc.getVelocityOpticalFlow(prev_img, curr_img));
		//ardroneの速度パラメータ変更

		//ドローンの制御部
		//オプティカルフローは未完成なので速度は(0,0)を渡してる
		ardrone.brain(pos, cv::Point2f(0,0), curr_img);
		

		//defaultの動きをする
		if (ardrone.getAvailable()){
			if (ardrone.default_move() == -1)
				break;
		}
		//ardroneのパラメータの方向に動く
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