#include "ardrone/ardrone.h"
#include "ardrone/drone.h"
#include <ctime>

// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------

int main(int argc, char *argv[])
{
	// AR.Drone class
	Drone ardrone;
	if (!ardrone.initialization())
		return -1;
	// Image of AR.Drone's camera
	cv::Mat image = ardrone.getImage();
	double vx, vy, vz, vr;
	while (1) {
		//ループごとにdroneの画像を取得
		image = ardrone.getImage();

		/*画像処理の部分*/
		/*
			.......
		*/

		/*制御部分*/
		/*
			.......
		*/

		//ardroneの速度パラメータ変更
		/*
		ardrone.setParameters(vx, vy, vz, vr);
		*/

		//defaultの動きをする
		if (ardrone.default_move() == -1)
			break;

		//ardroneのパラメータの方向に動く
		ardrone.Move();

		// Display the image
		cv::imshow("camera", image);
	}
	// See you
	ardrone.close();

	return 0;
}