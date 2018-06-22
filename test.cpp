#include "Aria.h"
#include "math.h"
#include "Target.h"
#include "Barrier.h"
#include <iostream>

using namespace std;

#ifndef PI
#define PI 3.1415926
#endif // !PI

#ifndef sonarNum
#define sonarNum 8
#endif // !sonarNum

#ifndef Target_N
#define Target_N 2
#endif

const float PositionError = 10.0;	//允许误差1cm
const float AngleError = 1.0;		//允许误差1度
const float SafeDistance = 100;	//修正方向距离(1.8m)，由误差允许范围计算得到，防止车辆在目标点附近转圈

extern int Barrier_Flag;
extern int Escape_Flag;
extern float BarrierThB, BarrierThE;	//开始及结束扫到障碍物时车辆转角
extern float BarrierDistance;			//与障碍物之间的距离

int main(int argc, char **argv)
{
	float anglePID[3] = { 0.5,0,0 };
	float positionPID[3] = { 0.5,0,0 };
	float current[3] = { 0,0,0 };
	float target[3] = { -2000,2000,0 };
	float ALL_Target[Target_N][3] = { { 2000,2000,0 },{3000,-1000,0} };
	ArSensorReading* sonarReading;
	float sonarDistance[sonarNum];
	//double reading, readingAngle;

	Aria::init();
	ArRobot robot;
	ArArgumentParser parser(&argc, argv);
	parser.loadDefaultArguments();
	ArLog::log(ArLog::Terse, "WARNING: this program does no sensing or avoiding of obstacles, the robot WILL collide with any objects in the way! Make sure the robot has approximately 3 meters of free space on all sides.");
	// ArRobotConnector connects to the robot, get some initial data from it such as type and name,
	// and then loads parameter files for this robot.
	ArRobotConnector robotConnector(&parser, &robot);
	if (!robotConnector.connectRobot())
	{
		ArLog::log(ArLog::Terse, "Could not connect to the robot.");
		if (parser.checkHelpAndWarnUnparsed())
		{
			Aria::logOptions();
			Aria::exit(1);
			return 1;
		}
	}
	if (!Aria::parseArgs())
	{
		Aria::logOptions();
		Aria::exit(1);
		return 1;
	}
	//set a target

	//sonar module
	ArSonarDevice sonar;

	robot.addRangeDevice(&sonar);	

	ArLog::log(ArLog::Normal, "Connected.");
	// Start the robot processing cycle running in the background.
	// True parameter means that if the connection is lost, then the 
	// run loop ends.
	robot.runAsync(true);
	// Print out some data from the SIP.  
	// We must "lock" the ArRobot object
	// before calling its methods, and "unlock" when done, to prevent conflicts
	// with the background thread started by the call to robot.runAsync() above.
	// See the section on threading in the manual for more about this.
	// Make sure you unlock before any sleep() call or any other code that will
	// take some time; if the robot remains locked during that time, then
	// ArRobot's background thread will be blocked and unable to communicate with
	// the robot, call tasks, etc.

	robot.lock();
	ArLog::log(ArLog::Normal, "Pose=(%.2f,%.2f,%.2f), Trans. Vel=%.2f, Rot. Vel=%.2f, Battery=%.2fV",
		robot.getX(), robot.getY(), robot.getTh(), robot.getVel(), robot.getRotVel(), robot.getBatteryVoltage());
	robot.unlock();
	// Sleep for 3 seconds.
	ArLog::log(ArLog::Normal, "Will start driving in 3 seconds...");
	ArUtil::sleep(3000);
	// The main part!!
	ArLog::log(ArLog::Normal, "Go test...");
	robot.lock();
	robot.enableMotors();

	//test
	//while (1)
	//{
	//	robot.setRotVel(10);
	//	robot.unlock();
	//	ArUtil::sleep(1000);
	//	robot.lock();
	//	ArLog::log(ArLog::Normal, "%.2f\n", robot.getTh());
	//}
	//**********************Section 1!!!*************************//
	//for (int i = 0; i < Target_N; i ++)
	//{
	//	for (int j = 0; j < 3; j++)
	//		target[j] = ALL_Target[i][j];
	//	Target* ProcessTarget = new Target(current, target);
	//	current[0] = robot.getX();
	//	current[1] = robot.getY();
	//	current[2] = robot.getTh();
	//	while (fabs(ProcessTarget->getAngleToTarget(current, target)) > AngleError)
	//	{
	//		current[2] = robot.getTh();
	//		robot.setRotVel(anglePID[0] * (ProcessTarget->getAngleToTarget(current, target)));
	//		robot.unlock();
	//		ArUtil::sleep(100);
	//		robot.lock();
	//		//ArLog::log(ArLog::Normal, "%.2f\n", simpletarget->getAngleToTarget(current, target));
	//	}
	//	while (ProcessTarget->getDistanceToTarget(current, target) > PositionError)
	//	{
	//		current[0] = robot.getX();
	//		current[1] = robot.getY();
	//		current[2] = robot.getTh();
	//		if(fabsf(ProcessTarget->getAngleToTarget(current, target)) < 90)	//判断车辆前进还是后退
	//			robot.setVel(positionPID[0] * (ProcessTarget->getDistanceToTarget(current, target)));
	//		else robot.setVel(-positionPID[0] * (ProcessTarget->getDistanceToTarget(current, target)));
	//		if (ProcessTarget->getDistanceToTarget(current, target) < SafeDistance)		//判断车辆是否需要修正方向
	//			robot.setRotVel(0);
	//		else robot.setRotVel(anglePID[0] * (ProcessTarget->getAngleToTarget(current, target)));
	//		robot.unlock();
	//		ArUtil::sleep(100);
	//		robot.lock();
	//	}
	//	ArLog::log(ArLog::Normal, "Arrive at the %d target!!", i + 1);
	//}
	////摆正车身
	//Target* FinalTarget = new Target(current, target);
	//while (fabsf(FinalTarget->getAngleToDir(current, target)) > AngleError)
	//{
	//	current[2] = robot.getTh();
	//	robot.setRotVel(anglePID[0] * (FinalTarget->getAngleToDir(current, target)));
	//	robot.unlock();
	//	ArUtil::sleep(100);
	//	robot.lock();
	//}

	//***************************Section 2*******************************//
	for (int i = 0; i < Target_N; i++)
	{
		for (int j = 0; j < 3; j++)
			target[j] = ALL_Target[i][j];
		Target* ProcessTarget = new Target(current, target);
		while (true)
		{
			current[0] = robot.getX();
			current[1] = robot.getY();
			current[2] = robot.getTh();
			while (fabs(ProcessTarget->getAngleToTarget(current, target)) > AngleError)
			{
				for (int i = 0; i < sonarNum; i++)
				{
					sonarReading = robot.getSonarReading(i);
					sonarDistance[i] = sonarReading->getRange();
				}
				current[0] = robot.getX();
				current[1] = robot.getY();
				current[2] = robot.getTh();
				robot.setRotVel(anglePID[0] * (ProcessTarget->getAngleToTarget(current, target)));
				robot.setVel(0);
				robot.unlock();
				ArUtil::sleep(100);
				robot.lock();
				if (Barrier_Flag == -1 && sonarDistance[2] < 4000) Barrier_Flag = 1;
				if (Barrier_Flag == -1 && sonarDistance[3] < 4000) Barrier_Flag = 2;
				if (Barrier_Flag == -1 && sonarDistance[2] > 4000) Barrier_Flag = 0;
				if (Barrier_Flag == -1 && sonarDistance[3] > 4000) Barrier_Flag = 0;
				if (Barrier_Flag == 0 && sonarDistance[2] < 4000)
				{
					Barrier_Flag = 1;
					BarrierThB = robot.getTh() - 5;
					if (BarrierThB < -180)
						BarrierThB += 360;
				}
				if (Barrier_Flag == 0 && sonarDistance[3] < 4000)
				{
					Barrier_Flag = 2;
					BarrierThB = robot.getTh() + 5;
					if (BarrierThB > 180)
						BarrierThB -= 360;
				}
				if (Barrier_Flag == 1 && sonarDistance[3] < 4000)
					Barrier_Flag = 3;
				if (Barrier_Flag == 2 && sonarDistance[2] < 4000)
					Barrier_Flag = 4;
				if (Barrier_Flag == 3 && sonarDistance[3] > 4000)
				{
					Barrier_Flag = 0;
					BarrierThE = robot.getTh() + 5;
					if (BarrierThE > 180)
						BarrierThE -= 360;
				}
				if (Barrier_Flag == 4 && sonarDistance[2] > 4000)
				{
					Barrier_Flag = 0;
					BarrierThE = robot.getTh() - 5;
					if (BarrierThE < 180)
						BarrierThE += 360;
				}
				BarrierDistance = (sonarDistance[2] + sonarDistance[3]) / 2;
				//SerachBarrier(robot, sonarDistance);
				//ArLog::log(ArLog::Normal, "%.2f\n", ProcessTarget->getAngleToTarget(current, target));
			}
			//矫正车辆方向，使得车辆所对的位置为障碍物的边界
			if (Barrier_Flag == 0 && Escape_Flag == 1 || BarrierDistance > ProcessTarget->getDistanceToTarget(current, target))
				break;						//当没有障碍物，或者障碍物的距离大于目标点的距离时，直走
			else if (Barrier_Flag > 0 && BarrierThB > -361)
			{
				float diff_angle;
				while (fabsf(BarrierThB - robot.getTh()) > AngleError)		//2号或3号超声波与障碍物边界重合
				{
					diff_angle = BarrierThB - robot.getTh();
					if (diff_angle > 180)
						diff_angle -= 360;
					else if(diff_angle < -180)
						diff_angle += 360;
					robot.setRotVel(anglePID[0] * diff_angle);
					robot.unlock();
					ArUtil::sleep(100);
					robot.lock();
				}
			}
			else if (Barrier_Flag == 2 || Barrier_Flag == 4 || ((Barrier_Flag == 1 || Barrier_Flag == 3) && BarrierThB == -361))	//2号或3号超声波与障碍物边界重合
				while (1)
				{
					for (int i = 0; i < sonarNum; i++)
					{
						sonarReading = robot.getSonarReading(i);
						sonarDistance[i] = sonarReading->getRange();
					}
					robot.setRotVel(20);
					robot.unlock();
					ArUtil::sleep(100);
					robot.lock();
					if (Barrier_Flag == -1 && sonarDistance[2] < 4000) Barrier_Flag = 1;
					if (Barrier_Flag == -1 && sonarDistance[3] < 4000) Barrier_Flag = 2;
					if (Barrier_Flag == -1 && sonarDistance[2] > 4000) Barrier_Flag = 0;
					if (Barrier_Flag == -1 && sonarDistance[3] > 4000) Barrier_Flag = 0;
					if (Barrier_Flag == 0 && sonarDistance[2] < 4000)
					{
						Barrier_Flag = 1;
						BarrierThB = robot.getTh() - 5;
						if (BarrierThB < -180)
							BarrierThB += 360;
					}
					if (Barrier_Flag == 0 && sonarDistance[3] < 4000)
					{
						Barrier_Flag = 2;
						BarrierThB = robot.getTh() + 5;
						if (BarrierThB > 180)
							BarrierThB -= 360;
					}
					if (Barrier_Flag == 1 && sonarDistance[3] < 4000)
						Barrier_Flag = 3;
					if (Barrier_Flag == 2 && sonarDistance[2] < 4000)
						Barrier_Flag = 4;
					if (Barrier_Flag == 3 && sonarDistance[3] > 4000)
					{
						Barrier_Flag = 0;
						BarrierThE = robot.getTh() + 5;
						if (BarrierThE > 180)
							BarrierThE -= 360;
					}
					if (Barrier_Flag == 4 && sonarDistance[2] > 4000)
					{
						Barrier_Flag = 0;
						BarrierThE = robot.getTh() - 5;
						if (BarrierThE < 180)
							BarrierThE += 360;
					}
					if (BarrierThE > -361)
						break;
				}
			//BarrierThE = -361;
			//BarrierThB = -361;
			//开始往障碍物边界行驶
			for (int i = 0; i < sonarNum; i++)
			{
				sonarReading = robot.getSonarReading(i);
				sonarDistance[i] = sonarReading->getRange();
			}
			EscapeBarrier(sonarDistance);
			while (Escape_Flag < 1)
			{
				for (int i = 0; i < sonarNum; i++)
				{
					sonarReading = robot.getSonarReading(i);
					sonarDistance[i] = sonarReading->getRange();
				}
				robot.setRotVel(0);
				robot.setVel(500);
				robot.unlock();
				ArUtil::sleep(100);
				robot.lock();
				EscapeBarrier(sonarDistance);
			}
			cout << "End a circle!!\n";
		}
		//调整车辆方向，朝向目标点
		//current[2] = robot.getTh();
		//while (fabs(ProcessTarget->getAngleToTarget(current, target)) > AngleError)
		//{
		//	current[0] = robot.getX();
		//	current[1] = robot.getY();
		//	current[2] = robot.getTh();
		//	robot.setRotVel(anglePID[0] * (ProcessTarget->getAngleToTarget(current, target)));
		//	robot.setVel(0);
		//	robot.unlock();
		//	ArUtil::sleep(100);
		//	robot.lock();
		//}
		//cout << fabs(ProcessTarget->getAngleToTarget(current, target));
		while (ProcessTarget->getDistanceToTarget(current, target) > PositionError)
		{
			for (int i = 0; i < sonarNum; i++)
			{
				sonarReading = robot.getSonarReading(i);
				sonarDistance[i] = sonarReading->getRange();
			}
			current[0] = robot.getX();
			current[1] = robot.getY();
			current[2] = robot.getTh();
			if (fabsf(ProcessTarget->getAngleToTarget(current, target)) < 90)	//判断车辆前进还是后退
				robot.setVel(positionPID[0] * (ProcessTarget->getDistanceToTarget(current, target)));
			else robot.setVel(-positionPID[0] * (ProcessTarget->getDistanceToTarget(current, target)));
			if (ProcessTarget->getDistanceToTarget(current, target) < SafeDistance)		//判断车辆是否需要修正方向
				robot.setRotVel(0);
			else robot.setRotVel(anglePID[0] * (ProcessTarget->getAngleToTarget(current, target)));

			robot.unlock();
			ArUtil::sleep(100);
			robot.lock();
			//ArLog::log(ArLog::Normal, "%.2f, %.2f\n%.2f\n", current[0],current[1],ProcessTarget->getDistanceToTarget(current, target));
		}
		ArLog::log(ArLog::Normal, "Arrive at the %d target!!", i + 1);
		//参数初始化
		Barrier_Flag = -1;
		Escape_Flag = 1;
		BarrierThB = -361, BarrierThE = -361;	//开始及结束扫到障碍物时车辆转角
		BarrierDistance = -1;			//与障碍物之间的距离
	}

	//摆正车身
	Target* FinalTarget = new Target(current, target);
	current[2] = robot.getTh();
	while (fabsf(FinalTarget->getAngleToDir(current, target)) > AngleError)
	{
		current[2] = robot.getTh();
		robot.setRotVel(anglePID[0] * (FinalTarget->getAngleToDir(current, target)));
		robot.unlock();
		ArUtil::sleep(100);
		robot.lock();
	}

	robot.setVel(0);
	robot.setRotVel(0);
	robot.unlock();
	ArUtil::sleep(1000);
	
	ArLog::log(ArLog::Normal, "Stopping.");
	robot.lock();
	robot.stop();
	robot.unlock();
	ArUtil::sleep(1000);
	// Other motion command functions include move(), setHeading(),
	// setDeltaHeading().  You can also adjust acceleration and deceleration
	// values used by the robot with setAccel(), setDecel(), setRotAccel(),
	// setRotDecel().  See the ArRobot class documentation for more.

	robot.lock();
	ArLog::log(ArLog::Normal, "Pose=(%.2f,%.2f,%.2f), Trans. Vel=%.2f, Rot. Vel=%.2f, Battery=%.2fV",
		robot.getX(), robot.getY(), robot.getTh(), robot.getVel(), robot.getRotVel(), robot.getBatteryVoltage());
	robot.unlock();

	ArLog::log(ArLog::Normal, "Ending robot thread...");
	robot.stopRunning();
	// wait for the thread to stop
	robot.waitForRunExit();
	// exit
	ArLog::log(ArLog::Normal, "Exiting.");
	Aria::exit(0);
	return 0;
}
