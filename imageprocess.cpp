#include <opencv2/opencv.hpp>
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

float vector_distance(cv::Point2f p1, cv::Point2f p2){
	return (p1 - p2).ddot(p1 - p2);
}
cv::Point2f calcCenter(std::list<cv::Point2f> data){
	float x = 0.0;
	float y = 0.0;
	std::list<cv::Point2f>::const_iterator it = data.begin();
	for (; it != data.end(); ++it) {
		x += it->x;
		y += it->y;
	}
	float ret_x = (data.size() == 0) ? 0 : x / static_cast<float>(data.size());
	float ret_y = (data.size() == 0) ? 0 : y / static_cast<float>(data.size());
	return cv::Point2f(ret_x, ret_y);
}

//完全分類じゃない。外乱は除くようにする
void Kmeans2(std::vector<cv::Point2f> data, std::vector<cv::Point2f> points, std::list<cv::Point2f> *ret){
	cv::Point2f center1 = data[0];
	cv::Point2f center2 = data[1];
	std::list<cv::Point2f> tmp[4];
	for (int i = 0; i < 20; i++){
		tmp[0].clear();
		tmp[1].clear();
		tmp[2].clear();
		tmp[3].clear();
		std::vector<cv::Point2f>::const_iterator it = data.begin();
		std::vector<cv::Point2f>::const_iterator pt = points.begin();
		for (; it != data.end(); ++it, ++pt){
			float dist = vector_distance(center1, *it);
			if (dist < vector_distance(center2, *it)){
				tmp[0].push_back(*it);
				tmp[1].push_back(*pt);
			}
			else{
				tmp[2].push_back(*it);
				tmp[3].push_back(*pt);
			}
		}
		center1 = calcCenter(tmp[0]);
		center2 = calcCenter(tmp[2]);
	}
	//外乱除去
	std::list<cv::Point2f>::const_iterator it = tmp[0].begin();
	std::list<cv::Point2f>::const_iterator pt = tmp[1].begin();
	cv::Point2f pcenter1 = calcCenter(tmp[1]);
	for (; it != tmp[0].end(); ++it, ++pt){
		//if (vector_distance(center1, *it) < 100 /*&& vector_distance(pcenter1, *pt) < 5000*/){
			ret[0].push_back(*it);
			ret[1].push_back(*pt);
		//}
	}
	std::list<cv::Point2f>::const_iterator itt = tmp[2].begin();
	std::list<cv::Point2f>::const_iterator ptt = tmp[3].begin();
	cv::Point2f pcenter2 = calcCenter(tmp[3]);
	for (; itt != tmp[2].end(); ++itt, ++ptt){
		//if (vector_distance(center1, *itt) < 100 /*&& vector_distance(pcenter2, *ptt) < 5000*/){
			ret[2].push_back(*itt);
			ret[3].push_back(*ptt);
		//}
	}
	/*std::list<cv::Point2f>::const_iterator it = ret[0].begin();
	std::list<cv::Point2f>::const_iterator pt = ret[1].begin();
	for (; it != ret[0].end(); ++it, ++pt){
		std::cout << *pt << endl;
	}*/
}

//opticalFlow
cv::Mat ImageProcess::OpticalFlow(cv::Mat _prev, cv::Mat _curr){
	cv::Mat prev = _prev.clone();
	cv::Mat curr = _curr.clone();
	cv::Mat color = _curr.clone();

	cv::cvtColor(prev, prev, CV_BGR2GRAY);
	cv::cvtColor(curr, curr, CV_BGR2GRAY);

	cv::Point2f CircleCenter = getPosCircleDetection(_curr);
//	if (CircleCenter != cv::Point2f(-1, -1)){
		std::vector<cv::Point2f> prev_pts;
		std::vector<cv::Point2f> curr_pts;


		cv::goodFeaturesToTrack(prev, prev_pts, 50, 0.01, 5);
		if (prev_pts.empty()) return curr;

		// Lucas-Kanadeメソッド＋画像ピラミッドに基づくオプティカルフロー
		cv::Mat status, error;
		cv::calcOpticalFlowPyrLK(prev, curr, prev_pts, curr_pts, status, error);
		if (curr_pts.empty()) return curr;

		// オプティカルフローの表示
		cv::Mat optflow;
		optflow = curr.clone();
		std::vector<cv::Point2f>::const_iterator p = prev_pts.begin();
		std::vector<cv::Point2f>::const_iterator n = curr_pts.begin();
		std::vector<cv::Point2f> velocity;
		std::list<cv::Point2f> clastering[4];
		for (; n != curr_pts.end(); ++n, ++p) {
			cv::Point2f dist = *p - *n;
			//移動距離が著しい点は除外する
			if (dist.ddot(dist) < 5000)
				velocity.push_back(dist);
		}
		if (velocity.size() <= 10) return curr;
		Kmeans2(velocity, prev_pts, clastering);
		//面積比を用いて物体分離
		
		std::list<cv::Point2f>::const_iterator it = clastering[0].begin();
		std::list<cv::Point2f>::const_iterator pt = clastering[1].begin();
		for (; it != clastering[0].end(); ++it, ++pt){
			if (clastering[0].size() < clastering[2].size()){
				if (vector_distance(CircleCenter, *pt) < 5000)
					cv::line(color, *pt, *pt + *it * 5, cv::Scalar(255, 0, 255), 2);
			}
			else{
				//cv::line(color, *pt, *pt + *it * 5, cv::Scalar(0, 255, 255), 2);
			}
		}
		//cerr << calcCenter(clastering[0]) << endl;
		/*if (!(calcCenter(clastering[0]).x == 0 && calcCenter(clastering[0]).y == 0)){
			cv::line(optflow, CircleCenter, calcCenter(clastering[0]) + CircleCenter - calcCenter(clastering[2]), cv::Scalar(255, 255, 255), 5);
			cv::circle(optflow, calcCenter(clastering[0]) + CircleCenter, 3, CV_RGB(0, 255, 0), CV_FILLED, 8, 0);
			cv::line(color, *pt, calcCenter(clastering[0]) + *pt - calcCenter(clastering[2]), cv::Scalar(0, 255, 255), 5);
		}*/
		std::list<cv::Point2f>::const_iterator itt = clastering[2].begin();
		std::list<cv::Point2f>::const_iterator ptt = clastering[3].begin();
		for (; itt != clastering[2].end(); ++itt, ++ptt){
			if (clastering[0].size() > clastering[2].size()){
				if (vector_distance(CircleCenter, *ptt) < 5000)
					cv::line(color, *ptt, *ptt + *itt * 5, cv::Scalar(255, 0, 255), 2);
			}
			else{
				//cv::line(color, *ptt, *ptt + *itt * 5, cv::Scalar(0, 255, 255), 2);
			}
		}
		//cerr << calcCenter(clastering[2]) << endl;
		/*if (!(calcCenter(clastering[2]).x == 0 && calcCenter(clastering[2]).y) == 0){
			cv::line(optflow, CircleCenter, calcCenter(clastering[2]) + CircleCenter - calcCenter(clastering[0]), cv::Scalar(255, 255, 255), 5);
			cv::circle(optflow, calcCenter(clastering[2]) + CircleCenter, 3, CV_RGB(0, 255, 0), CV_FILLED, 8, 0);
			cv::line(color, *ptt, calcCenter(clastering[2]) + *pt - calcCenter(clastering[0]), cv::Scalar(255, 255, 0), 5);
		}*/
//	}
	return curr;
}

cv::Mat ImageProcess::FaceDetection(cv::Mat _image){
	cv::Mat image = _image.clone();
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

cv::Mat ImageProcess::Labeling(cv::Mat _image){
	cv::Mat image = _image.clone();
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

cv::Mat ImageProcess::CircleDetection(cv::Mat _image){
	cv::Mat image = _image.clone();
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

cv::Mat ImageProcess::LineDetection(cv::Mat _image){
	cv::Mat image = _image.clone();
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

cv::Point2f ImageProcess::getPosCircleDetection(cv::Mat _image){
	cv::Mat image = _image.clone();
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
	cv::Point genuine_center = cv::Point(-1,-1);
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

	imshow("circle", image);
	return cv::Point2f(x,y);
}

cv::Point2f ImageProcess::getVelocityOpticalFlow(cv::Mat _prev, cv::Mat _curr){
	cv::Mat prev = _prev.clone();
	cv::Mat curr = _curr.clone();
	cv::Mat color = _curr.clone();
	cv::Mat optflow;
	optflow = curr.clone();

	cv::cvtColor(prev, prev, CV_BGR2GRAY);
	cv::cvtColor(curr, curr, CV_BGR2GRAY);

	cv::Point2f CircleCenter = getPosCircleDetection(_curr);
	if (CircleCenter != cv::Point2f(-1, -1)){
		std::vector<cv::Point2f> prev_pts;
		std::vector<cv::Point2f> curr_pts;


		cv::goodFeaturesToTrack(prev, prev_pts, 50, 0.01, 5);
		if (prev_pts.empty()) return cv::Point2f(0, 0);

		// Lucas-Kanadeメソッド＋画像ピラミッドに基づくオプティカルフロー
		cv::Mat status, error;
		cv::calcOpticalFlowPyrLK(prev, curr, prev_pts, curr_pts, status, error);
		if (curr_pts.empty()) return cv::Point2f(0, 0);

		// オプティカルフローの表示
		std::vector<cv::Point2f>::const_iterator p = prev_pts.begin();
		std::vector<cv::Point2f>::const_iterator n = curr_pts.begin();
		std::vector<cv::Point2f> velocity;
		std::list<cv::Point2f> clastering[4];
		for (; n != curr_pts.end(); ++n, ++p) {
			cv::Point2f dist = *n - *p;
			//移動距離が著しい点は除外する
			if (dist.ddot(dist) < 5000)
				velocity.push_back(dist);
		}
		if (velocity.size() <= 10) return cv::Point2f(0, 0);
		Kmeans2(velocity, prev_pts, clastering);
		//面積比を用いて物体分離

		if (clastering[0].size() < clastering[2].size()){
			std::list<cv::Point2f>::const_iterator it = clastering[0].begin();
			std::list<cv::Point2f>::const_iterator pt = clastering[1].begin();
			/*for (; it != clastering[0].end(); ++it, ++pt){
				cv::line(optflow, *pt, *pt + *it*5, cv::Scalar(255, 255, 255), 2);
				}*/
			//cerr << calcCenter(clastering[0]) << endl;
			if (!(calcCenter(clastering[0]).x == 0 && calcCenter(clastering[0]).y)){
				cv::line(optflow, CircleCenter, calcCenter(clastering[0]) + CircleCenter - calcCenter(clastering[2]), cv::Scalar(255, 255, 255), 5);
				cv::circle(optflow, calcCenter(clastering[0]) + CircleCenter, 3, CV_RGB(0, 255, 0), CV_FILLED, 8, 0);
			}
			imshow("optflow", optflow);
			return calcCenter(clastering[0]);
		}
		else{
			std::list<cv::Point2f>::const_iterator itt = clastering[2].begin();
			std::list<cv::Point2f>::const_iterator ptt = clastering[3].begin();
			/*for (; itt != clastering[2].end(); ++itt, ++ptt){
				cv::line(optflow, *ptt, *ptt + *itt*5, cv::Scalar(255, 255, 255), 2);
				}*/
			//cerr << calcCenter(clastering[2]) << endl;
			if (!(calcCenter(clastering[2]).x == 0 && calcCenter(clastering[2]).y)){
				cv::line(optflow, CircleCenter, calcCenter(clastering[2]) + CircleCenter - calcCenter(clastering[0]), cv::Scalar(255, 255, 255), 5);
				cv::circle(optflow, calcCenter(clastering[2]) + CircleCenter, 3, CV_RGB(0, 255, 0), CV_FILLED, 8, 0);
			}
			imshow("optflow", optflow);
			return calcCenter(clastering[2]);
		}
	}
	else{
		cv::imshow("optflow", optflow);
		return cv::Point2f(0, 0);
	}
}

void ImageProcess::DrawLine(cv::Mat &image, cv::Point2f from, cv::Point2f to){
	cv::Mat img = image.clone();
	cv::line(img, from, to, cv::Scalar(255, 0, 255), 2);
	cv::circle(img, to, 3, CV_RGB(0, 255, 0), CV_FILLED, 8, 0);
	cv::imshow("line-draw", img);
}