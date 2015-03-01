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
	while (1) {
		image = ardrone.getImage();
		//default‚Ì“®‚«‚ğ‚·‚é
		if (ardrone.default_move() == -1)
			break;

		//ardrone‚Ìƒpƒ‰ƒ[ƒ^‚Ì•ûŒü‚É“®‚­
		ardrone.Move();
		// Display the image
		cv::imshow("camera", image);
	}
	// See you
	ardrone.close();

	return 0;
}