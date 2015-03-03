#include "ardrone\ardrone.h"
#include "imageprocess.h"
#include "Labeling.h"

#include <iostream>
using namespace std;

int ImageProcess::useVideoCapture(){
	cap.open(0);
	if (cap.isOpened()){
		//fps10�Ŏg���悤�ɂ���
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

	// ������
	cv::Size flowSize(30, 30);
	cv::Point2f center = cv::Point(prev.cols / 2., prev.rows / 2.);
	for (int i = 0; i<flowSize.width; ++i) {
		for (int j = 0; j<flowSize.width; ++j) {
			cv::Point2f p(i*float(prev.cols) / (flowSize.width - 1),
				j*float(prev.rows) / (flowSize.height - 1));
			prev_pts.push_back((p - center)*0.9f + center);
		}
	}

	// Lucas-Kanade���\�b�h�{�摜�s���~�b�h�Ɋ�Â��I�v�e�B�J���t���[
	cv::Mat status, error;
	cv::calcOpticalFlowPyrLK(prev, curr, prev_pts, curr_pts, status, error);

	// �I�v�e�B�J���t���[�̕\��
	cv::Mat optflow;
	optflow = curr.clone();
	std::vector<cv::Point2f>::const_iterator p = prev_pts.begin();
	std::vector<cv::Point2f>::const_iterator n = curr_pts.begin();
	for (; n != curr_pts.end(); ++n, ++p) {
		cv::Point2f dist = *p - *n;
		//�ړ��������������_�͏��O����
		if (dist.ddot(dist) < 5000)
			cv::line(optflow, *p, *n, cv::Scalar(150, 0, 0), 2);
	}

	return optflow;
}

cv::Mat ImageProcess::FaceDetection(cv::Mat image){
	double scale = 4.0;
	cv::cvtColor(image, image, CV_BGR2GRAY);

	// �������ԒZ�k�̂��߂ɉ摜���k��
	cv::Mat smallimage(cv::saturate_cast<int>(image.rows / scale), cv::saturate_cast<int>(image.cols / scale), CV_8UC1);
	cv::resize(image, smallimage, smallimage.size(), 0, 0, cv::INTER_LINEAR);
	cv::equalizeHist(smallimage, smallimage);
	
	// ���ފ�̓ǂݍ���
	std::string cascadeName = "haarcascade_frontalface_alt.xml"; // Haar-like
	//std::string cascadeName = "./lbpcascade_frontalface.xml"; // LBP
	cv::CascadeClassifier cascade;
	if (!cascade.load(cascadeName))
		return cv::Mat();

	std::vector<cv::Rect> faces;
	// �}���`�X�P�[���i��j�T��
	// �摜�C�o�͋�`�C�k���X�P�[���C�Œ��`���C�i�t���O�j�C�ŏ���`
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

	// ���ʂ̕`��
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
	// Labeling�̌��ʂ��󂯎��
	cv::Mat label(image.size(), CV_16SC1);
	LabelingBS labeling;
	labeling.Exec(bin.data, (short *)label.data, bin.cols, bin.rows, false, 0);
	cout << "Regions:" << labeling.GetNumOfRegions() << endl;
	return bin;
}