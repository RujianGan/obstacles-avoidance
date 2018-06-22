#pragma once
#include "Aria.h"
#ifndef sonarNum
#define sonarNum 8
#endif // !sonarNum

extern int Barrier_Flag;
extern int Escape_Flag;
extern float BarrierThB, BarrierThE;	//开始及结束扫到障碍物时车辆转角
extern float BarrierDistance;			//与障碍物之间的距离

void SerachBarrier(ArRobot robot, float sonarDistance[sonarNum]);
void EscapeBarrier(float sonarDistance[sonarNum]);
