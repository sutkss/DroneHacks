#include "ardrone/ardrone.h"
#include <iostream>
#include <ctime>

using namespace std;
using namespace cv;

// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------
int main(int argc, char *argv[])
{
	VideoCapture cap(0); // デフォルトカメラをオープン
	if (!cap.isOpened())  // 成功したかどうかをチェック
		return -1;

	// 前のフレームを保存しておく
	Mat prev;
	cap >> prev;
	cv::cvtColor(prev, prev, CV_BGR2GRAY);
	while (waitKey(1) == -1)
	{
		// 現在のフレームを保存
		Mat curr;
		cap >> curr;
		cv::cvtColor(curr, curr, CV_BGR2GRAY);

		std::vector<cv::Point2f> prev_pts;
		std::vector<cv::Point2f> curr_pts;

		// 初期化
		cv::Size flowSize(30, 30);
		cv::Point2f center = cv::Point(prev.cols / 2., prev.rows / 2.);
		for (int i = 0; i<flowSize.width; ++i) {
			for (int j = 0; j<flowSize.width; ++j) {
				cv::Point2f p(i*float(prev.cols) / (flowSize.width - 1),
					j*float(prev.rows) / (flowSize.height - 1));
				prev_pts.push_back((p - center)*0.9f + center);
			}
		}

		// Lucas-Kanadeメソッド＋画像ピラミッドに基づくオプティカルフロー
		cv::Mat status, error;
		cv::calcOpticalFlowPyrLK(prev, curr, prev_pts, curr_pts, status, error);

		// オプティカルフローの表示
		Mat optflow;
		optflow = curr.clone();
		std::vector<cv::Point2f>::const_iterator p = prev_pts.begin();
		std::vector<cv::Point2f>::const_iterator n = curr_pts.begin();
		for (; n != curr_pts.end(); ++n, ++p) {
			cv::Point2f dist = *p - *n;
			//移動距離が著しい点は除外する
			if (dist.ddot(dist) < 1000)
				cv::line(optflow, *p, *n, cv::Scalar(150, 0, 0), 2);
		}
		
		cv::imshow("optflow", optflow);

		// 前のフレームを保存
		prev = curr;
	}
	// VideoCapture デストラクタにより，カメラは自動的に終了処理されます
	
	return 0;
}