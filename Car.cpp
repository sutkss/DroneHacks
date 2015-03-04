#include "Car.h"
#include "imageprocess.h"

Car::Car(){
	x = 0.0;
	y = 0.0;
	vx = 0.0;
	vy = 0.0;
}
void Car::calcPosition(cv::Point2f p){
	x = p.x;
	y = p.y;
}
void Car::calcVelocity(){

}
void Car::Move(){

}