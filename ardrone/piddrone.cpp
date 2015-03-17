#include "ardrone.h"
#include "drone.h"
#include "piddrone.h"
#include <fstream>

PIDDrone::PIDDrone(){
	using namespace std;
	ifstream csvFile_x(para_x);
	int i = 0;
	double _Ax[num_x*num_x];
	double _Bx[num_x];
	double _Cx[2 * num_x];
	double _Dx[2];
	double _Kx[num_x];
	double _Lx[num_x * 2];
	double _Axd[num_x*num_x];
	double _Bxd[num_x];
	double _Kxd[num_x];
	double _Lxd[num_x * 2];

	double DATA_x[size_x];
	for (i = 0; i < size_x; i++){
		DATA_x[i] = 0;
		getline(csvFile_x.seekg(0, ios_base::cur), str, ',');
		ss.str(str);
		ss >> DATA_x[i];
		ss.str("");
		ss.clear(stringstream::goodbit);
	}
	int length_ = 0;
	for (int i = 0; i < 10; i++)
	{
		switch (i){
		case 0:
			length = num_x*num_x;
			for (int j = 0; j < length; j++){
				_Ax[j] = DATA_x[j];
			}
			length_ = length_ + length;
			break;
		case 1:
			length = num_x;
			for (int j = 0; j < length; j++){
				_Bx[j] = DATA_x[length_ + j];
			}
			length_ = length_ + length;
			break;
		case 2:
			length = 2 * num_x;
			for (int j = 0; j < length; j++){
				_Cx[j] = DATA_x[length_ + j];
			}
			length_ = length_ + length;
			break;
		case 3:
			length = 2;
			for (int j = 0; j < length; j++){
				_Dx[j] = DATA_x[length_ + j];
			}
			length_ = length_ + length;
			break;
		case 4:
			length = num_x;
			for (int j = 0; j < length; j++){
				_Kx[j] = DATA_x[length_ + j];
			}
			length_ = length_ + length;
			break;
		case 5:
			length = num_x * 2;
			for (int j = 0; j < length; j++){
				_Lx[j] = DATA_x[length_ + j];
			}
			length_ = length_ + length;
			break;
		case 6:
			length = num_x*num_x;
			for (int j = 0; j < length; j++){
				_Axd[j] = DATA_x[length_ + j];
			}
			length_ = length_ + length;
			break;
		case 7:
			length = num_x;
			for (int j = 0; j < length; j++){
				_Bxd[j] = DATA_x[length_ + j];
			}
			length_ = length_ + length;
			break;
		case 8:
			length = num_x;
			for (int j = 0; j < length; j++){
				_Kxd[j] = DATA_x[length_ + j];
			}
			length_ = length_ + length;
			break;
		case 9:
			length = num_x * 2;
			for (int j = 0; j < length; j++){
				_Lxd[j] = DATA_x[length_ + j];
			}
			break;
		}
	}
	csvFile_x.close();
	Ax = cv::Mat(num_x, num_x, CV_64FC1, _Ax).clone();
	Bx = cv::Mat(num_x, 1, CV_64FC1, _Bx).clone();
	Cx = cv::Mat(2, num_x, CV_64FC1, _Cx).clone();
	Dx = cv::Mat(2, 1, CV_64FC1, _Dx).clone();
	Kx = cv::Mat(1, num_x, CV_64FC1, _Kx).clone();
	Lx = cv::Mat(num_x, 2, CV_64FC1, _Lx).clone();
	Axd = cv::Mat(num_x, num_x, CV_64FC1, _Axd).clone();
	Bxd = cv::Mat(num_x, 1, CV_64FC1, _Bxd).clone();
	Kxd = cv::Mat(1, num_x, CV_64FC1, _Kxd).clone();
	Lxd = cv::Mat(num_x, 2, CV_64FC1, _Lxd).clone();

	ifstream csvFile_y(para_y);

	double _Ay[num_x*num_x];
	double _By[num_x];
	double _Cy[2 * num_x];
	double _Dy[2];
	double _Ky[num_x];
	double _Ly[num_x * 2];
	double _Ayd[num_x*num_x];
	double _Byd[num_x];
	double _Kyd[num_x];
	double _Lyd[num_x * 2];

	double DATA_y[size_x];

	for (i = 0; i < size_x; i++){
		DATA_y[i] = 0;
		getline(csvFile_y.seekg(0, ios_base::cur), str, ',');
		ss.str(str);
		ss >> DATA_y[i];
		ss.str("");
		ss.clear(stringstream::goodbit);
	}
	length_ = 0;
	for (int i = 0; i < 10; i++)
	{
		switch (i){
		case 0:
			length = num_x*num_x;
			for (int j = 0; j < length; j++){
				_Ay[j] = DATA_y[j];
			}
			length_ = length_ + length;
			break;
		case 1:
			length = num_x;
			for (int j = 0; j < length; j++){
				_By[j] = DATA_y[length_ + j];
			}
			length_ = length_ + length;
			break;
		case 2:
			length = 2 * num_x;
			for (int j = 0; j < length; j++){
				_Cy[j] = DATA_y[length_ + j];
			}
			length_ = length_ + length;
			break;
		case 3:
			length = 2;
			for (int j = 0; j < length; j++){
				_Dy[j] = DATA_y[length_ + j];
			}
			length_ = length_ + length;
			break;
		case 4:
			length = num_x;
			for (int j = 0; j < length; j++){
				_Ky[j] = DATA_y[length_ + j];
			}
			length_ = length_ + length;
			break;
		case 5:
			length = num_x * 2;
			for (int j = 0; j < length; j++){
				_Ly[j] = DATA_y[length_ + j];
			}
			length_ = length_ + length;
			break;
		case 6:
			length = num_x*num_x;
			for (int j = 0; j < length; j++){
				_Ayd[j] = DATA_y[length_ + j];
			}
			length_ = length_ + length;
			break;
		case 7:
			length = num_x;
			for (int j = 0; j < length; j++){
				_Byd[j] = DATA_y[length_ + j];
			}
			length_ = length_ + length;
			break;
		case 8:
			length = num_x;
			for (int j = 0; j < length; j++){
				_Kyd[j] = DATA_y[length_ + j];
			}
			length_ = length_ + length;
			break;
		case 9:
			length = num_x * 2;
			for (int j = 0; j < length; j++){
				_Lyd[j] = DATA_y[length_ + j];
			}
			break;
		}
	}

	csvFile_y.close();

	Ay = cv::Mat(num_x, num_x, CV_64FC1, _Ay).clone();
	By = cv::Mat(num_x, 1, CV_64FC1, _By).clone();
	Cy = cv::Mat(2, num_x, CV_64FC1, _Cy).clone();
	Dy = cv::Mat(2, 1, CV_64FC1, _Dy).clone();
	Ky = cv::Mat(1, num_x, CV_64FC1, _Ky).clone();
	Ly = cv::Mat(num_x, 2, CV_64FC1, _Ly).clone();
	Ayd = cv::Mat(num_x, num_x, CV_64FC1, _Ayd).clone();
	Byd = cv::Mat(num_x, 1, CV_64FC1, _Byd).clone();
	Kyd = cv::Mat(1, num_x, CV_64FC1, _Kyd).clone();
	Lyd = cv::Mat(num_x, 2, CV_64FC1, _Lyd).clone();

	ifstream csvFile_z(para_z);

	const int size_z = num_z*num_z + num_z + 2 * num_z + 2 + num_z + num_z * 2 + num_z*num_z + num_z + num_z + num_z * 2;
	double _Az[num_z*num_z];
	double _Bz[num_z];
	double _Cz[2 * num_z];
	double _Dz[2];
	double _Kz[num_z];
	double _Lz[num_z * 2];
	double _Azd[num_z*num_z];
	double _Bzd[num_z];
	double _Kzd[num_z];
	double _Lzd[num_z * 2];

	double DATA_z[size_z];

	for (i = 0; i < size_z; i++){
		DATA_z[i] = 0;
		getline(csvFile_z.seekg(0, ios_base::cur), str, ',');
		ss.str(str);
		ss >> DATA_z[i];
		ss.str("");
		ss.clear(stringstream::goodbit);
	}
	length_ = 0;
	for (int i = 0; i < 10; i++)
	{
		switch (i){
		case 0:
			length = num_z*num_z;
			for (int j = 0; j < length; j++){
				_Az[j] = DATA_z[j];
			}
			length_ = length_ + length;
			break;
		case 1:
			length = num_z;
			for (int j = 0; j < length; j++){
				_Bz[j] = DATA_z[length_ + j];
			}
			length_ = length_ + length;
			break;
		case 2:
			length = 2 * num_z;
			for (int j = 0; j < length; j++){
				_Cz[j] = DATA_z[length_ + j];
			}
			length_ = length_ + length;
			break;
		case 3:
			length = 2;
			for (int j = 0; j < length; j++){
				_Dz[j] = DATA_z[length_ + j];
			}
			length_ = length_ + length;
			break;
		case 4:
			length = num_z;
			for (int j = 0; j < length; j++){
				_Kz[j] = DATA_z[length_ + j];
			}
			length_ = length_ + length;
			break;
		case 5:
			length = num_z * 2;
			for (int j = 0; j < length; j++){
				_Lz[j] = DATA_z[length_ + j];
			}
			length_ = length_ + length;
			break;
		case 6:
			length = num_z*num_z;
			for (int j = 0; j < length; j++){
				_Azd[j] = DATA_z[length_ + j];
			}
			length_ = length_ + length;
			break;
		case 7:
			length = num_z;
			for (int j = 0; j < length; j++){
				_Bzd[j] = DATA_z[length_ + j];
			}
			length_ = length_ + length;
			break;
		case 8:
			length = num_z;
			for (int j = 0; j < length; j++){
				_Kzd[j] = DATA_z[length_ + j];
			}
			length_ = length_ + length;
			break;
		case 9:
			length = num_z * 2;
			for (int j = 0; j < length; j++){
				_Lzd[j] = DATA_z[length_ + j];
			}
			break;
		}
	}

	csvFile_z.close();

	Az = cv::Mat(num_z, num_z, CV_64FC1, _Az).clone();
	Bz = cv::Mat(num_z, 1, CV_64FC1, _Bz).clone();
	Cz = cv::Mat(2, num_z, CV_64FC1, _Cz).clone();
	Dz = cv::Mat(2, 1, CV_64FC1, _Dz).clone();
	Kz = cv::Mat(1, num_z, CV_64FC1, _Kz).clone();
	Lz = cv::Mat(num_z, 2, CV_64FC1, _Lz).clone();
	Azd = cv::Mat(num_z, num_z, CV_64FC1, _Azd).clone();
	Bzd = cv::Mat(num_z, 1, CV_64FC1, _Bzd).clone();
	Kzd = cv::Mat(1, num_z, CV_64FC1, _Kzd).clone();
	Lzd = cv::Mat(num_z, 2, CV_64FC1, _Lzd).clone();

	start = cv::getTickCount();
	last = cv::getTickCount();

	GetLocalTime(&st);
}

void PIDDrone::PIDControl(cv::Point2f pos){
	if (pos.x != -1 && pos.y != -1 && pos.x*pos.x+pos.y*pos.y < 5000)
		piddone = 1;
	std::cout << piddone << std::endl;
	if (piddone == 0){
		double x2 = pos.x;
		double y2 = pos.y;

		// date取得
		z = getAltitude();
		double vx, vy, vz;
		double _velocity = getVelocity(&vx, &vy, &vz);
		cv::Mat velocity(3, 1, CV_64FC1, _velocity);
		double pitch = getPitch();
		double roll = getRoll();
		double yaw = getYaw() - yaw_start;

		// 時間
		double time = (cv::getTickCount() - start) / cv::getTickFrequency();

		double dt = (cv::getTickCount() - last) / cv::getTickFrequency();
		last = cv::getTickCount();

		if (x2 == -1 && x1 == -1){
			x2 = x1;
			y2 = y1;
		}
		else{
			x2 = -x2 + 180;
			y2 = y2 - 360;
			double sigma1, sigma2, sigma3;
			sigma1 = 2 * tan(beta) / 360 * x2*cos(pitch) + 2 * tan(alpha) / 640 * y2*sin(pitch)*sin(roll) + sin(pitch)*cos(roll);
			sigma2 = 2 * tan(alpha) / 640 * y2*cos(roll) - sin(roll);
			sigma3 = 2 * tan(beta) / 360 * x2*sin(pitch) - 2 * tan(alpha) / 640 * y2*cos(pitch)*sin(roll) - cos(pitch)*cos(roll);
			h = z;

			x2 = -(-d*cos(pitch) + sigma1 / sigma3*(d*sin(pitch) - h));
			y2 = -(sigma2 / sigma3*(d*sin(pitch) - h));
		}

		x1 = x2;
		y1 = y2;

		// Rotation matrices
		double _RX[] = { 1.0, 0.0, 0.0,
			0.0, cos(roll), sin(roll),
			0.0, -sin(roll), cos(roll) };
		double _RY[] = { cos(pitch), 0.0, -sin(pitch),
			0.0, 1.0, 0.0,
			sin(pitch), 0.0, cos(pitch) };
		double _RZ[] = { cos(0), -sin(0), 0.0,
			sin(0), cos(0), 0.0,
			0.0, 0.0, 1.0 };
		cv::Mat RX(3, 3, CV_64FC1, _RX);
		cv::Mat RY(3, 3, CV_64FC1, _RY);
		cv::Mat RZ(3, 3, CV_64FC1, _RZ);

		// 速度の計算（絶対座標系）
		double _V[] = { vx, vy, vz };
		cv::Mat V(3, 1, CV_64FC1, _V);
		V = RX*RY*RZ*V;

		// Dead reckoning
		_P = _P + (V + V_last) / 2 * dt;

		V_last = V;

		//double _output_x[] = { V.at<double>(0, 0), _P.at<double>(0, 0) };
		double _output_x[] = { V.at<double>(0, 0), x2 };
		cv::Mat output_x3(2, 1, CV_64FC1, _output_x);

		//double _output_y[] = { V.at<double>(1, 0), _P.at<double>(1, 0) };
		double _output_y[] = { V.at<double>(1, 0), y2 };
		cv::Mat output_y3(2, 1, CV_64FC1, _output_y);

		double _output_z[] = { V.at<double>(2, 0), z - ref[2] };
		cv::Mat output_z3(2, 1, CV_64FC1, _output_z);

		if (fabs(_output_z[1]) < 0.03){
			Mode = 1;
		}


		cv::Mat _input_x = cv::Mat::zeros(1, 1, CV_64FC1);
		cv::Mat _input_y = cv::Mat::zeros(1, 1, CV_64FC1);
		cv::Mat _input_z = cv::Mat::zeros(1, 1, CV_64FC1);

		//// 連続時間の状態推定（4次のRunge-Kutta法による状態方程式の数値計算）
		//k1 = (Ax - Lx*Cx)*state_est_x1 + Bx*input_x1 + Lx*output_x1;
		//k2 = (Ax - Lx*Cx)*(state_est_x1 + dt*k1) + Bx*input_x2 + Lx*output_x2;
		//k3 = (Ax - Lx*Cx)*(state_est_x1 + dt*k2) + Bx*input_x2 + Lx*output_x2;
		//k4 = (Ax - Lx*Cx)*(state_est_x1 + 2 * dt*k3) + Bx*input_x3 + Lx*output_x3;
		//state_est_x3 = state_est_x1 + dt / 3 * (k1 + 2 * k2 + 2 * k3 + k4);


		//k1 = (Ay - Ly*Cy)*state_est_y1 + By*input_y1 + Ly*output_y1;
		//k2 = (Ay - Ly*Cy)*(state_est_y1 + dt*k1) + By*input_y2 + Ly*output_y2;
		//k3 = (Ay - Ly*Cy)*(state_est_y1 + dt*k2) + By*input_y2 + Ly*output_y2;
		//k4 = (Ay - Ly*Cy)*(state_est_y1 + 2 * dt*k3) + By*input_y3 + Ly*output_y3;
		//state_est_y3 = state_est_y1 + dt / 3 * (k1 + 2 * k2 + 2 * k3 + k4);

		//k1z = (Az - Lz*Cz)*state_est_z1 + Bz*input_z1 + Lz*output_z1;
		//k2z = (Az - Lz*Cz)*(state_est_z1 + dt*k1z) + Bz*input_z2 + Lz*output_z2;
		//k3z = (Az - Lz*Cz)*(state_est_z1 + dt*k2z) + Bz*input_z2 + Lz*output_z2;
		//k4z = (Az - Lz*Cz)*(state_est_z1 + 2 * dt*k3z) + Bz*input_z3 + Lz*output_z3;
		//state_est_z3 = state_est_z1 + dt / 3 * (k1z + 2 * k2z + 2 * k3z + k4z);

		// 離散時間の状態推定
		state_est_x3 = (Axd - Lxd*Cx)*state_est_x2 + Lxd*output_x3 + Bxd*input_x3;
		state_est_y3 = (Ayd - Lyd*Cy)*state_est_y2 + Lyd*output_y3 + Byd*input_y3;
		state_est_z3 = (Azd - Lzd*Cz)*state_est_z2 + Lzd*output_z3 + Bzd*input_z3;



		// Move
		if (!onGround() && z > 0.6){
			////// 連続時間モデルにおける入力
			//_input_x = -Kx*state_est_x3*Mode;
			//_input_y = -Ky*state_est_y3*Mode;
			//_input_z = -Kz*state_est_z3;

			// 離散時間モデルにおける入力
			_input_x = -Kxd*state_est_x3*(double)Mode;
			_input_y = -Kyd*state_est_y3*(double)Mode;
			_input_z = -Kzd*state_est_z3;

			input_x3 = _input_x.at<double>(0, 0);
			input_y3 = _input_y.at<double>(0, 0);
			input_z3 = _input_z.at<double>(0, 0);
			input_yaw = 0;

			if (fabs(input_x3) > 0.08){
				input_x3 = input_x3 / fabs(input_x3)*0.08;
			}
			if (fabs(input_y3) > 0.08){
				input_y3 = input_y3 / fabs(input_y3)*0.08;
			}
			if (z < 1.7)
				input_z2 = 0.3;
			if (z > 2.1)
				input_z3 = -0.3;

			//if (sqrt(_output_x[1] * _output_x[1] + _output_y[1] * _output_y[1]) < 0.1){
			//	input_x3 = 0;
			//	input_y3 = 0;
			//}
			move3D2(&input_x3, &input_y3, &input_z3, &input_yaw);
			//setvx(input_x3);
			//setvy(input_y3);
			//setvz(input_z3);
		}

		state_est_x1 = state_est_x2;
		state_est_x2 = state_est_x3;
		output_x1 = output_x2;
		output_x2 = output_x3;
		input_x1 = input_x2;
		input_x2 = input_x3;

		state_est_y1 = state_est_y2;
		state_est_y2 = state_est_y3;
		output_y1 = output_y2;
		output_y2 = output_y3;
		input_y1 = input_y2;
		input_y2 = input_y3;

		state_est_z1 = state_est_z2;
		state_est_z2 = state_est_z3;
		output_z1 = output_z2;
		output_z2 = output_z3;
		input_z1 = input_z2;
		input_z2 = input_z3;
	}
}

void PIDDrone::move3D2(double* vx, double* vy, double* vz, double* vr)
{
	// Command velocities
	float v[4] = { *vy, *vx, *vz, *vr };
	int mode = (fabs(v[0]) > 0.0 || fabs(v[1]) > 0.0 || fabs(v[2]) > 0.0 || fabs(v[3]) > 0.0);

	double vz_max = 2000 * 0.001;
	double euler_angle_max = 0.52;
	double yaw_max = 6.11;

	// max velocity
	double v_max[4] = { euler_angle_max, euler_angle_max, vz_max, yaw_max };

	// Nomarization (-1.0 to +1.0)
	int i;
	for (i = 0; i < 4; i++) {
		v[i] = v[i] / v_max[i];
		if (fabs(v[i]) > 1.0) v[i] = v[i] / fabs(v[i]);
	}

	*vx = v[1] * euler_angle_max;
	*vy = v[0] * euler_angle_max;
	*vz = v[2] * vz_max;

	// Send a command
	if (mutexCommand) pthread_mutex_lock(mutexCommand);
	sockCommand.sendf("AT*PCMD=%d,%d,%d,%d,%d,%d\r", ++seq, mode, *(int*)(&v[0]), *(int*)(&v[1]), *(int*)(&v[2]), *(int*)(&v[3]));
	if (mutexCommand) pthread_mutex_unlock(mutexCommand);
}
