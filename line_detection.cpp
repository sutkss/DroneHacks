#include "ardrone/ardrone.h"
#include <ctime>

using namespace cv;

#define epsilon 0.1
#define delta 1.0
// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------

double max(double a, double b){
	if (a > b){
		return a;
	}
	return b;
}

double min(double a, double b){
	if (a < b){
		return a;
	}
	return b;
}

int main(int argc, char *argv[])
{
	VideoCapture cap(0); // デフォルトカメラをオープン
	cap.set(CV_CAP_PROP_FPS, 30);
	if (!cap.isOpened())  // 成功したかどうかをチェック
		return -1;

	for (;;)
	{
		// 元の画像
		Mat frame;
		// 線検出用画像
		Mat gray_img;
		// エッジ検出用画像
		Mat edge_img;
		// 二値化画像
		Mat bin_img;
		
		// カメラから新しいフレームを取得
		cap >> frame;
		
		// gray_imgにframeのモノクロ画像
		cvtColor(frame, gray_img, CV_BGR2GRAY);

		// ヒストグラム平坦化
		// cv::equalizeHist(gray_img, gray_img);

		// 二値化
		threshold(gray_img, bin_img, 75, 255, THRESH_BINARY );

		// エッジ検出
		Canny(bin_img,edge_img,50,200,3);
		
		// 古典的Hough変換
		std::vector<cv::Vec2f> lines;
		cv::HoughLines(edge_img, lines, 1, CV_PI / 180, 200, 0, 0);
		std::vector<cv::Vec2f>::iterator it = lines.begin();
		for(; it!=lines.end(); ++it) {
		float rho1 = (*it)[0], theta1 = (*it)[1];
		double a1 = cos(theta1), b1 = sin(theta1);
		double x1 = a1*rho1, y1 = b1*rho1;
		cv::Point pt1, pt2;
		pt1.x = cv::saturate_cast<int>(x1 + 1000*(-b1));
		pt1.y = cv::saturate_cast<int>(y1 + 1000*(a1));
		pt2.x = cv::saturate_cast<int>(x1 - 1000*(-b1));
		pt2.y = cv::saturate_cast<int>(y1 - 1000*(a1));
		cv::line(frame, pt1, pt2, cv::Scalar(0, 0, 255), 3, CV_AA);
		}
		

		// 確率的Hough変換
		/*
		std::vector<cv::Vec4i> lines;
		cv::HoughLinesP(edge_img,lines,1,CV_PI/180,200,0,100);
	
		for (size_t i = 0; i < lines.size(); i++){
			line(frame, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0, 0, 255), 3, 8);
		}
		*/
		
		cv::imshow("frame",frame);
		cv::imshow("binary", bin_img);
		cv::imshow("モノクロ", gray_img);
		cv::imshow("edge", edge_img);
		if (waitKey(30) >= 0) break;
	}

	return 0;
}