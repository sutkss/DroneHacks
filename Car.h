#pragma once
#include "imageprocess.h"
class Car{
public:
	//�p�����[�^�͂Q�����̌��݈ʒu�A���x
	//�ʒu�̓h���[���ɑ΂��Ă̑��Έʒu
	//���x�͑��x�x�N�g��
	//x,y���̎����͍������Ă���摜�̎������ɂ���
	double x, y;
	double vx, vy;
	Car();
	void calcPosition(cv::Point2f p);
	void calcVelocity();
	void Move();
};