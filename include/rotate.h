#pragma once
#ifndef _ROTATE_H
#define _ROTATE_H
#include <iostream>
#include "timer.h"
#include "Drone_data.h"
#include "Set_Altitude.h"
#include "border_detection_task24.h"

extern msr::airlib::MultirotorRpcLibClient client;
extern void set_control_cmd(bool, int, int, float, float, float, float, float, float, float);
extern bool flag_H;	
extern cv::Mat down_image;

//让四轴旋转180度，旋转完成返回1
void turn180()
{
	bool succ = false;
	//yar_rate=0.7，duration=PI/yar_rate
	do {
		succ = client.moveByAngleThrottle(0.0f, 0.0f, 0.587402f, 0.7f, 4.45);
	} while (!succ);
	//set_control_cmd(true, 1, 1, .0f, .0f, .0f, .0f, 4.45, .0f, -0.7f);	//llf增加了flag_set_altitude
	//std::this_thread::sleep_for(std::chrono::duration<double>(0.1f));// 睡眠0.1
}

//从10非往角落
void flyToCorner()
{
	
	
	while (true)
	{
		/*b_flag = border_dectection_task2(down_image);
		if (b_flag.bottom == 1)
		{
			break;
		}*/
		set_control_cmd(true, 1, 1, .0f, .0f, .0f, .0f, 0.05f, .0f, 0.7f);
	}
}
#endif  _ROTATE_H

