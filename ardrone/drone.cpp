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
	//double vx = 0.0, vy = 0.0, vz = 0.0, vr = 0.0;
	if (key == 0x260000) vx = 0.2;
	if (key == 0x280000) vx = -0.2;
	if (key == 0x250000) vy = 0.2;
	if (key == 0x270000) vy = -0.2;
	if (key == 'q')      vz = 0.3;
	if (key == 'a')      vz = -0.3;

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

void Drone::brain(cv::Point2f pos, cv::Point2f v, cv::Mat img){
	double vx, vy, vr, vz;
	vr = 0;
	vz = 0;
	getVelocity(&vx, &vy, &vz);
	//円がなかった
	if (pos.x == -1 && pos.y == -1){
		vx = 0;
		vy = 0;
	}
	else{
		//円を見つけた
		//画像の座標系と実際のdroneの座標系を変換
		double target_y = -1 * (pos.x - img.cols / 2.0);
		double target_x = -1 * (pos.y - img.rows / 2.0);

		//droneの速度決定する
		vx = target_x / (sqrt(img.rows*img.rows/4.0) * 5);
		vy = target_y / (sqrt(img.cols*img.cols/4.0) * 5);
	}
	std::cout << vx << " " << vy << std::endl;
	setParameters(vx, vy, 0.0, 0.0);
}
