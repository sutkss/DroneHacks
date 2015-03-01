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
		//���[�v���Ƃ�drone�̉摜���擾
		image = ardrone.getImage();

		/*�摜�����̕���*/
		/*
			.......
		*/

		/*���䕔��*/
		/*
			.......
		*/

		//ardrone�̑��x�p�����[�^�ύX
		/*
		ardrone.setParameters(vx, vy, vz, vr);
		*/

		//default�̓���������
		if (ardrone.default_move() == -1)
			break;

		//ardrone�̃p�����[�^�̕����ɓ���
		ardrone.Move();

		// Display the image
		cv::imshow("camera", image);
	}
	// See you
	ardrone.close();

	return 0;
}