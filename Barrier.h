#pragma once
#include "Aria.h"
#ifndef sonarNum
#define sonarNum 8
#endif // !sonarNum

extern int Barrier_Flag;
extern int Escape_Flag;
extern float BarrierThB, BarrierThE;	//��ʼ������ɨ���ϰ���ʱ����ת��
extern float BarrierDistance;			//���ϰ���֮��ľ���

void SerachBarrier(ArRobot robot, float sonarDistance[sonarNum]);
void EscapeBarrier(float sonarDistance[sonarNum]);
