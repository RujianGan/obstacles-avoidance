#include "Target.h"

#ifndef PI
#define PI 3.1415926
#endif // !PI

Target::Target()
{
	float current[3] = { 0,0,0 };
	float target[3] = { 2000,2000,0 };
	Target(current, target);
}

Target::Target(float current[3], float target[3])
{
	this->current[0] = current[0];
	this->current[1] = current[1];
	this->current[2] = current[2];
	this->target[0] = target[0];
	this->target[1] = target[1];
	this->target[2] = target[2];
	this->Diatance = sqrtf(powf(target[1] - current[1], 2) + powf(target[0] - current[0], 2));
}

Target::~Target()
{

}

void Target::setTarget(float target[3])
{
	this->target[0] = target[0];
	this->target[1] = target[1];
	this->target[2] = target[2];
}

float Target::getAngleToTarget(float current[3], float target[3])
{
	float angle;
	angle = atan2(target[1] - current[1], target[0] - current[0]) * 180 / PI - current[2];
	while (angle >= 180.0)
		angle -= 360.0;
	while (angle < -180.0)
		angle += 360.0;
	return angle;
}

float Target::getAngleToDir(float current[3], float target[3])
{
	if (target[2] - current[2] > 180)
		return target[2] - current[2] - 360;
	else if (target[2] - current[2] < -180)
		return target[2] - current[2] + 360;
}

float Target::getDistanceToTarget(float current[3], float target[3])
{
	return sqrtf(powf(target[1] - current[1], 2) + powf(target[0] - current[0], 2));

}