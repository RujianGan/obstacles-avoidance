#pragma once
#include "Aria.h"


class Target
{
public:
	Target();
	//0: X; 1: Y; 2: Th
	Target(float current[3], float target[3]);
	~Target();
	void setTarget(float target[3]);
	float getAngleToTarget(float current[3], float target[3]);
	float getAngleToDir(float current[3], float target[3]);
	float getDistanceToTarget(float current[3], float target[3]);
	float Diatance;
private:
	float current[3];
	float target[3];
};