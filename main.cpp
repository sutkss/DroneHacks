#include "ardrone/ardrone.h"
using namespace cv;
// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    // AR.Drone class
    ARDrone ardrone;

    // Initialize
    if (!ardrone.open()) {
        std::cout << "Failed to initialize." << std::endl;
        return -1;
    }

    // Battery
    std::cout << "Battery = " << ardrone.getBatteryPercentage() << "[%]" << std::endl;

    // Instructions
    std::cout << "***************************************" << std::endl;
    std::cout << "*       CV Drone sample program       *" << std::endl;
    std::cout << "*           - How to play -           *" << std::endl;
    std::cout << "***************************************" << std::endl;
    std::cout << "*                                     *" << std::endl;
    std::cout << "* - Controls -                        *" << std::endl;
    std::cout << "*    'Space' -- Takeoff/Landing       *" << std::endl;
    std::cout << "*    'Up'    -- Move forward          *" << std::endl;
    std::cout << "*    'Down'  -- Move backward         *" << std::endl;
    std::cout << "*    'Left'  -- Turn left             *" << std::endl;
    std::cout << "*    'Right' -- Turn right            *" << std::endl;
    std::cout << "*    'Q'     -- Move upward           *" << std::endl;
    std::cout << "*    'A'     -- Move downward         *" << std::endl;
    std::cout << "*                                     *" << std::endl;
    std::cout << "* - Others -                          *" << std::endl;
    std::cout << "*    'C'     -- Change camera         *" << std::endl;
    std::cout << "*    'Esc'   -- Exit                  *" << std::endl;
    std::cout << "*                                     *" << std::endl;
    std::cout << "***************************************\n" << std::endl;

	Mat kao;

    while (1) {
        // Key input
        int key = cv::waitKey(33);
        if (key == 0x1b) break;

        // Get an image
        //cv::Mat image = ardrone.getImage();

		Mat frame = ardrone.getImage();
		cvtColor(frame, kao, CV_BGR2GRAY);

		// 処理時間短縮のために画像を縮小
		//Mat smallkao(cv::saturate_cast<int>(kao.rows / scale), cv::saturate_cast<int>(kao.cols / scale), CV_8UC1);
		//cv::resize(kao, smallkao, smallkao.size(), 0, 0, cv::INTER_LINEAR);
		//cv::equalizeHist(smallkao, smallkao);
		cv::equalizeHist(kao, kao);

		// 分類器の読み込み
		std::string cascadeName = "haarcascade_frontalface_alt.xml"; // Haar-like
		//std::string cascadeName = "./lbpcascade_frontalface.xml"; // LBP
		cv::CascadeClassifier cascade;
		if (!cascade.load(cascadeName))
			return -1;

		std::vector<cv::Rect> faces;
		// マルチスケール（顔）探索
		// 画像，出力矩形，縮小スケール，最低矩形数，（フラグ），最小矩形
		/*cascade.detectMultiScale(smallkao, faces,
		1.1, 2,
		CV_HAAR_SCALE_IMAGE
		,
		cv::Size(30, 30));*/
		cascade.detectMultiScale(kao, faces,
			1.1, 2,
			CV_HAAR_SCALE_IMAGE
			,
			cv::Size(30, 30));

		// 結果の描画
		std::vector<cv::Rect>::const_iterator r = faces.begin();
		
		// Move
		double vx = 0.0, vy = 0.0, vz = 0.0, vr = 0.0;
		int radiusm = 0;
		for (; r != faces.end(); ++r) {
			cv::Point center;
			int radius;
			//center.x = cv::saturate_cast<int>((r->x + r->width*0.5)*scale);
			//center.y = cv::saturate_cast<int>((r->y + r->height*0.5)*scale);
			//radius = cv::saturate_cast<int>((r->width + r->height)*0.25*scale);
			center.x = cv::saturate_cast<int>((r->x + r->width*0.5));
			center.y = cv::saturate_cast<int>((r->y + r->height*0.5));
			radius = cv::saturate_cast<int>((r->width + r->height)*0.25);
			if (radius > radiusm){
				
				radiusm = radius;

				if (center.x < kao.size().width/ 3){
					vr = 1.0;
				}
				else if (center.x < kao.size().width * 2 / 3){
					vr = 0.0;
				}
				else {
					vr = -1.0;
				}

				if (center.y < kao.size().height / 3){
					vz = 1.0;
				}
				else if (center.y < kao.size().height * 2 / 3){
					vz = 0.0;
				}
				else {
					vz = -1.0;
				}
				std::cout << "center.x:" << center.x << std::endl;
				std::cout << "center.y:" << center.y << std::endl;
			}
			cv::circle(kao, center, radius, cv::Scalar(80, 80, 255), 3, 8, 0);
		}

		//cv::namedWindow("result", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
		//cv::imshow("result", kao);

		if (waitKey(30) >= 0) break;
        // Take off / Landing 
        if (key == ' ') {
            if (ardrone.onGround()) ardrone.takeoff();
            else                    ardrone.landing();
        }

        
		/*
        if (key == 0x260000) vx = 1.0;
        if (key == 0x280000) vx = -1.0;
        if (key == 0x250000) vr = 1.0;
        if (key == 0x270000) vr = -1.0;
        if (key == 'q')      vz = 1.0;
        if (key == 'a')      vz = -1.0;
        */
		ardrone.move3D(vx, vy, vz, vr);
		std::cout << "vr:" << vr << std::endl;
		std::cout << "vz:" << vz << std::endl;

		std::cout << "rows:" << kao.size().width << std::endl;
		std::cout << "cols:" << kao.size().height << std::endl;


        // Change camera
        static int mode = 0;
        if (key == 'c') ardrone.setCamera(++mode % 4);

        // Display the image
        //cv::imshow("camera", image);
		cv::imshow("camera", kao);
    }

    // See you
    ardrone.close();

    return 0;
}