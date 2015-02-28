#include "ardrone/ardrone.h"
#include <ctime>

using namespace cv;

// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------
int main(int argc, char *argv[])
{
	VideoCapture cap(0); // デフォルトカメラをオープン
	cap.set(CV_CAP_PROP_FPS, 1);
	if (!cap.isOpened())  // 成功したかどうかをチェック
		return -1;

	for (;;)
	{
		Mat frame;
		cap >> frame; // カメラから新しいフレームを取得

		cv::namedWindow("result", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
		cv::imshow("result", frame);

		if (waitKey(30) >= 0) break;
	}
	// VideoCapture デストラクタにより，カメラは自動的に終了処理されます

	return 0;
}