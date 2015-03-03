#include "ardrone\ardrone.h"
#include "imageprocess.h"
#include "Labeling.h"

#include <iostream>
using namespace std;

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

cv::Mat ImageProcess::FaceDetection(cv::Mat image){
	double scale = 4.0;
	cv::cvtColor(image, image, CV_BGR2GRAY);

	// 処理時間短縮のために画像を縮小
	cv::Mat smallimage(cv::saturate_cast<int>(image.rows / scale), cv::saturate_cast<int>(image.cols / scale), CV_8UC1);
	cv::resize(image, smallimage, smallimage.size(), 0, 0, cv::INTER_LINEAR);
	cv::equalizeHist(smallimage, smallimage);
	
	// 分類器の読み込み
	std::string cascadeName = "haarcascade_frontalface_alt.xml"; // Haar-like
	//std::string cascadeName = "./lbpcascade_frontalface.xml"; // LBP
	cv::CascadeClassifier cascade;
	if (!cascade.load(cascadeName))
		return cv::Mat();

	std::vector<cv::Rect> faces;
	// マルチスケール（顔）探索
	// 画像，出力矩形，縮小スケール，最低矩形数，（フラグ），最小矩形
	/*cascade.detectMultiScale(smallkao, faces,
	1.1, 2,
	CV_HAAR_SCALE_IMAGE
	,
	cv::Size(30, 30));*/
	cascade.detectMultiScale(smallimage, faces,
		1.1, 2,
		CV_HAAR_SCALE_IMAGE
		,
		cv::Size(30, 30));

	// 結果の描画
	std::vector<cv::Rect>::const_iterator r = faces.begin();
	for (; r != faces.end(); ++r) {
		cv::Point center;
		int radius;
		center.x = cv::saturate_cast<int>((r->x + r->width*0.5)*scale);
		center.y = cv::saturate_cast<int>((r->y + r->height*0.5)*scale);
		radius = cv::saturate_cast<int>((r->width + r->height)*0.25*scale);
		cv::circle(image, center, radius, cv::Scalar(80, 80, 255), 3, 8, 0);
	}
	return image;
}

cv::Mat ImageProcess::Labeling(cv::Mat image){
	cv::Mat bin;
	cv::cvtColor(image, bin, CV_BGR2GRAY);
	cv::GaussianBlur(bin, bin, cv::Size(11, 11), 10, 10);
	cv::threshold(bin, bin, 10, 255, cv::THRESH_BINARY);
	cv::erode(bin, bin, cv::Mat(), cv::Point(-1, -1), 10);
	cv::dilate(bin, bin, cv::Mat(), cv::Point(-1, -1), 5);
	// Labelingの結果を受け取る
	cv::Mat label(image.size(), CV_16SC1);
	LabelingBS labeling;
	labeling.Exec(bin.data, (short *)label.data, bin.cols, bin.rows, false, 0);
	cout << "Regions:" << labeling.GetNumOfRegions() << endl;
	return bin;
}

cv::Mat ImageProcess::CircleDetection(cv::Mat image){
	// 円検出用画像
	cv::Mat gray_img;
	// 円検出後比較用画像
	cv::Mat tmp_img;
	cv::Mat cmp_img;

	double x, y;// 円の中心座標

	cvtColor(image, gray_img, CV_BGR2GRAY);
	cvtColor(image, cmp_img, CV_BGR2GRAY);
	// エッジ検出
	Canny(cmp_img, cmp_img, 50, 200);
	cmp_img = ~cmp_img;

	// ヒストグラム平坦化
	cv::equalizeHist(gray_img, gray_img);

	// 平滑化
	cv::GaussianBlur(gray_img, gray_img, cv::Size(11, 11), 11, 11);

	// 円検出
	std::vector<cv::Vec3f> circles;
	cv::HoughCircles(gray_img, circles, CV_HOUGH_GRADIENT, 1, 100, 100, 50);
	cv::Point center;
	cv::Point genuine_center;
	int radius;
	int genuine_radius = 0;
	std::vector<cv::Vec3f>::iterator it = circles.begin();
	double ep = 100.0;
	for (; it != circles.end(); ++it) {
		radius = cv::saturate_cast<int>((*it)[2]);
		center = cv::Point(cv::saturate_cast<int>((*it)[0]), cv::saturate_cast<int>((*it)[1]));

		// 円検出後に比較
		cvtColor(image, tmp_img, CV_BGR2GRAY);
		cv::circle(tmp_img, center, radius, cv::Scalar(0, 0, 0), 2);
		// エッジ検出
		Canny(tmp_img, tmp_img, 50, 200);
		tmp_img = ~tmp_img;
		if (cv::matchShapes(tmp_img, cmp_img, CV_CONTOURS_MATCH_I2, 0) < ep){
			genuine_center = center;
			genuine_radius = radius;
			ep = cv::matchShapes(tmp_img, cmp_img, CV_CONTOURS_MATCH_I2, 0);
		}
	}

	// 円描画
	cv::circle(image, genuine_center, genuine_radius, cv::Scalar(0, 0, 255), 2);
	// 円中心描画
	cv::circle(image, genuine_center, 0, cv::Scalar(0, 255, 0), 2);

	x = genuine_center.x;
	y = genuine_center.y;

	return image;
}

cv::Mat ImageProcess::LineDetection(cv::Mat image){
	cv::Mat gray_img;
	// エッジ検出用画像
	cv::Mat edge_img;
	// 二値化画像
	cv::Mat bin_img;

	// gray_imgにimageのモノクロ画像
	cvtColor(image, gray_img, CV_BGR2GRAY);

	// ヒストグラム平坦化
	// cv::equalizeHist(gray_img, gray_img);

	// 二値化
	threshold(gray_img, bin_img, 75, 255, cv::THRESH_BINARY);

	// エッジ検出
	Canny(bin_img, edge_img, 50, 200, 3);

	// 古典的Hough変換
	std::vector<cv::Vec2f> lines;
	cv::HoughLines(edge_img, lines, 1, CV_PI / 180, 200, 0, 0);
	std::vector<cv::Vec2f>::iterator it = lines.begin();
	for (; it != lines.end(); ++it) {
		float rho1 = (*it)[0], theta1 = (*it)[1];
		double a1 = cos(theta1), b1 = sin(theta1);
		double x1 = a1*rho1, y1 = b1*rho1;
		cv::Point pt1, pt2;
		pt1.x = cv::saturate_cast<int>(x1 + 1000 * (-b1));
		pt1.y = cv::saturate_cast<int>(y1 + 1000 * (a1));
		pt2.x = cv::saturate_cast<int>(x1 - 1000 * (-b1));
		pt2.y = cv::saturate_cast<int>(y1 - 1000 * (a1));
		cv::line(image, pt1, pt2, cv::Scalar(0, 0, 255), 3, CV_AA);
	}
	return image;
}