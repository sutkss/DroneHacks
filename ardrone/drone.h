#include "ardrone.h"
#pragma once
class Drone : public ARDrone{
private:
	//���x�p�����[�^
	double vx;
	double vy;
	double vz;
	double vr;
	//Drone�̃��\�[�X�����p�\���ǂ���
	bool available;
public:
	/*�p�����[�^������*/
	Drone();
	/*�h���[���ɐڑ��Ƃ��̏���������*/
	bool initialization();
	/*�f�t�H���g�ł������L�[����̓����̏���*/
	int default_move();
	/*���x�p�����[�^�̐ݒ菈��*/
	void setParameters(double _vx, double _vy, double _vz, double _vr);
	/*���x�p�����[�^�̎擾����*/
	double getvx(){ return vx; }
	double getvy(){ return vy; }
	double getvz(){ return vz; }
	double getvr(){ return vr; }
	/*�p�����[�^�̕����ɓ���*/
	void Move();
	void brain(cv::Point2f pos, cv::Point2f v, cv::Mat img);
	bool getAvailable(){
		return available;
	}
	~Drone(){
		if (available)
			close();
	}
};