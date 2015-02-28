/*#include "ardrone/ardrone.h"
#include <ctime>

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
	// Image of AR.Drone's camera
	cv::Mat image = ardrone.getImage();

	// Video name
	std::time_t t = std::time(NULL);
	std::tm *local = std::localtime(&t);
	std::ostringstream stream;
	stream << 1900 + local->tm_year << "-" << 1 + local->tm_mon << "-" << local->tm_mday << "-" << local->tm_hour << "-" << local->tm_min << "-" << local->tm_sec << ".avi";

	cv::VideoWriter writer(stream.str(), cv::VideoWriter::fourcc('D', 'I', 'B', ' '), 30, cv::Size(image.cols, image.rows));

	while (1) {
		// Key input
		int key = cv::waitKey(33);
		if (key == 0x1b) break;

		// Get an image
		cv::Mat image = ardrone.getImage();

		// Take off / Landing 
		if (key == ' ') {
			if (ardrone.onGround()) ardrone.takeoff();
			else                    ardrone.landing();
		}

		// Move
		double vx = 0.0, vy = 0.0, vz = 0.0, vr = 0.0;
		if (key == 0x260000) vx = 1.0;
		if (key == 0x280000) vx = -1.0;
		if (key == 0x250000) vr = 1.0;
		if (key == 0x270000) vr = -1.0;
		if (key == 'q')      vz = 1.0;
		if (key == 'a')      vz = -1.0;
		ardrone.move3D(vx, vy, vz, vr);

		// Change camera
		static int mode = 0;
		if (key == 'c') ardrone.setCamera(++mode % 4);

		// Display the image
		cv::imshow("camera", image);

		// Write a frame
		writer << image;
	}

	// See you
	ardrone.close();

	return 0;
}*/