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
	VideoCapture cap(0); // デフォルトカメラをオープン
	cap.set(CV_CAP_PROP_FPS, 30);
	if (!cap.isOpened())  // 成功したかどうかをチェック
		return -1;

	for (;;)
	{
		// 元の画像
		Mat frame;
		// 円検出用画像
		Mat gray_img;
		// 円検出後比較用画像
		Mat tmp_img;
		Mat cmp_img;

		double x, y;// 円の中心座標

		//Mat frame2;	// test用
		cap >> frame; // カメラから新しいフレームを取得
		cvtColor(frame, gray_img, CV_BGR2GRAY);
		//cvtColor(frame, tmp_img, CV_BGR2GRAY);
		cvtColor(frame, cmp_img, CV_BGR2GRAY);
		//cvtColor(frame, frame2, CV_BGR2GRAY);
		// エッジ検出
		Canny(cmp_img, cmp_img, 50, 200);
		cmp_img = ~cmp_img;

		// ヒストグラム平坦化
		cv::equalizeHist(gray_img,gray_img);

		// 平滑化
		cv::GaussianBlur(gray_img,gray_img, cv::Size(11, 11), 11, 11);

		// 円検出
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

			// 検出した円を描画
			//cv::circle(frame2, center, radius, cv::Scalar(255, 0, 0), 2);
			
			// 円検出後に比較
			cvtColor(frame, tmp_img, CV_BGR2GRAY);
			cv::circle(tmp_img, center, radius, cv::Scalar(0, 0, 0), 2);
				// エッジ検出
			Canny(tmp_img, tmp_img, 50, 200);
			tmp_img = ~tmp_img;
			if (cv::matchShapes(tmp_img, cmp_img, CV_CONTOURS_MATCH_I2, 0) < ep ){
				genuine_center = center;
				genuine_radius = radius;
				ep = cv::matchShapes(tmp_img, cmp_img, CV_CONTOURS_MATCH_I2, 0);
			}
		}
		
		// 円描画
		cv::circle(frame, genuine_center, genuine_radius, cv::Scalar(0, 0, 255), 2);
		// 円中心描画
		cv::circle(frame, genuine_center, 0, cv::Scalar(0, 255, 0), 2);
		
		x = genuine_center.x;
		y = genuine_center.y;

		cv::imshow("circle", frame);
		//cv::imshow("frame2", frame2);
		if (waitKey(30) >= 0) break;
	}

	return 0;
}