#include "Barrier.h"
#include "Aria.h"

#ifndef sonarNum
#define sonarNum 8
#endif // !sonarNum

int Barrier_Flag = -1;
int Escape_Flag = 1;
float BarrierThB = -361, BarrierThE = -361;	//开始及结束扫到障碍物时车辆转角
float BarrierDistance = -1;			//与障碍物之间的距离

void SerachBarrier(ArRobot robot, float *sonarDistance)
{
	if (Barrier_Flag == -1 && sonarDistance[2] < 4000)
		Barrier_Flag = 1;
	if (Barrier_Flag == 0 && sonarDistance[2] < 4000)
	{
		Barrier_Flag = 1;
		BarrierThB = robot.getTh();
	}
	if (Barrier_Flag == 1 && sonarDistance[3] < 4000)
		Barrier_Flag = 2;
	if (Barrier_Flag == 2 && sonarDistance[3] > 4000)
	{
		Barrier_Flag = 0;
		BarrierThE = robot.getTh();
	}
	/*if (sonarDistance[3] < 4000 && sonarDistance[2] < 4000)
		BarrierDistance = (sonarDistance[3] + sonarDistance[2]) / 2;
	else if (sonarDistance[3] > 4000 && sonarDistance[2] > 4000)
		BarrierDistance = 0;
	else BarrierDistance = sonarDistance[2] < sonarDistance[3] ? sonarDistance[2] : sonarDistance[3];*/
}

void EscapeBarrier(float sonarDistance[sonarNum])
{
	if (Escape_Flag == 0 && (sonarDistance[0] > 4000 && sonarDistance[5] > 4000))
	{
		Escape_Flag = 1;
		Barrier_Flag = 0;
	}
	if (Barrier_Flag > 0 && (sonarDistance[0] > 4000 && sonarDistance[5] > 4000)) Escape_Flag = -1;
	if ((sonarDistance[0] < 4000 || sonarDistance[5] < 4000))
		Escape_Flag = 0;
}