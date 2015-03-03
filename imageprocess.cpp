#include "ardrone\ardrone.h"
#include "imageprocess.h"

int ImageProcess::useVideoCapture(){
	cap.open(0);
	if (cap.isOpened()){
		//fps10で使うようにする
		cap.set(CV_CAP_PROP_FPS, 10);
		return 0;
	}
	else
		return -1;
}

cv::Mat ImageProcess::getVideoCapture(){
	tmp_img = cv::Mat();
	if (cap.isOpened()){
		cap >> tmp_img;
	}
	return tmp_img;
}

//opticalFlow
cv::Mat ImageProcess::OpticalFlow(cv::Mat prev, cv::Mat curr){
	cv::cvtColor(prev, prev, CV_BGR2GRAY);
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
	cv::Mat optflow;
	optflow = curr.clone();
	std::vector<cv::Point2f>::const_iterator p = prev_pts.begin();
	std::vector<cv::Point2f>::const_iterator n = curr_pts.begin();
	for (; n != curr_pts.end(); ++n, ++p) {
		cv::Point2f dist = *p - *n;
		//移動距離が著しい点は除外する
		if (dist.ddot(dist) < 5000)
			cv::line(optflow, *p, *n, cv::Scalar(150, 0, 0), 2);
	}

	return optflow;
}