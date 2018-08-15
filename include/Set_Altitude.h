#pragma once
#ifndef _SET_ALLTITUDE_H
#define _SET_ALLTITUDE_H
#include <iostream>
#include "Drone_data.h"
#include "PID.h"

extern msr::airlib::MultirotorRpcLibClient client;
extern struct control_cmd control_cmdset;
extern void set_control_cmd(bool, int, int, float, float, float, float, float, float, float);

//对定高指令的封装
//target_altitude---目标高度，mode---定高方式：1表示精调，2表示粗调
void controlAltitude(float target_altitude, int mode)
{
	set_control_cmd(true, 3, mode, .0f, .0f, .0f, .0f, .05f, target_altitude, 0.0f);
}

//定高细节的实现
/*
返回值：0---定到目标高度，正常退出    1---没有定到目标高度，异常退出
target_attlitude---目标高度
mode---定高方式：1表示精调   2表示粗调
*/
int setAltitude(float target_altitude, int mode)
{
	int result_set_attlitude = 1;
	float allowable_error;	//允许的误差
	float current_error;	//cirrent_error=target_attlitude-current_attlitude
	PID PID_temp;

	if (mode == 1)
	{
		allowable_error = 1.0;
	}
	else
	{
		allowable_error = 2.0;
	}

	//最多执行20次，否则异常退出
	int i = 0;
	while(true)
	{
		std::cout << "第" << (i++) << "次adjust altitude" << endl;
		current_error = target_altitude - Barometer_data.altitude;

		if (abs(current_error) < allowable_error)
		{
			result_set_attlitude = 0;
			break;
		}

		//控制高度
		double throttle_increment = PID_temp.PIDZ(target_altitude, allowable_error);	//得到的throttle_increment是加速度
		if (throttle_increment > allowable_error*0.1 || throttle_increment < (-allowable_error*0.1))
		{
			client.moveByAngleThrottle(0.0f, 0.0f, 0.5875 + throttle_increment, 0.0f, 0.7f);
		}
	}

	return result_set_attlitude;
}

#endif _SET_ALLTITUDE_H
