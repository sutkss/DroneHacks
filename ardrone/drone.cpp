#include "ardrone.h"
#include "drone.h"

Drone::Drone(){
	vx = 0.0;
	vy = 0.0;
	vz = 0.0;
	vr = 0.0;
	available = false;
};

/*ドローンとの接続に成功したらtrue,そうでなければfalseを返す*/
bool Drone::initialization(){
	if (!open()){
		available = false;
		return false;
	}
	else{
		// Battery
		std::cout << "Battery = " << getBatteryPercentage() << "[%]" << std::endl;

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
	}
	available = true;
	return true;
}

/*Escキーが押されて終了したいときに-1を返す、そうでなければ0を返す*/
int Drone::default_move(){
	int key = cv::waitKey(33);
	if (key == 0x1b) return -1;

	// Take off / Landing 
	if (key == ' ') {
		if (onGround()) takeoff();
		else landing();
	}

	// Move
	double vx = 0.0, vy = 0.0, vz = 0.0, vr = 0.0;
	if (key == 0x260000) vx = 1.0;
	if (key == 0x280000) vx = -1.0;
	if (key == 0x250000) vr = 1.0;
	if (key == 0x270000) vr = -1.0;
	if (key == 'q')      vz = 1.0;
	if (key == 'a')      vz = -1.0;

	// Change camera
	static int mode = 0;
	if (key == 'c') setCamera(++mode % 4);
	return 0;
}

/*引数で渡されたパラメータに設定する*/
void Drone::setParameters(double _vx, double _vy, double _vz, double _vr){
	vx = _vx;
	vy = _vy;
	vz = _vz;
	vr = _vr;
}

void Drone::Move(){
	move3D(vx, vy, vz, vr);
}