
/*
 * main.cpp
 *
 *  Created on: June 18, 2016
 *      Author: ljw
 */
#ifndef PID_H__
#define PID_H__
#include "PIDController.h"
#define CLIP3(_n1, _n, _n2)                                                    \
  {                                                                            \
    if (_n < _n1)                                                              \
      _n = _n1;                                                                \
    if (_n > _n2)                                                              \
      _n = _n2;                                                                \
  }

extern double kp, ki, kd;
extern double vkp, vki, vkd;

//extern PIDController PID_position_x;//位置控制PID
//extern PIDController PID_position_y;//位置控制PID

class PID {
public:
	//PID(ROSThread &thread, FindRob &find_rob): thread_(thread), 
	//    find_rob_(find_rob), lasterrorx_(0), lasterrory_(0) {
	//  
	//  double vkp = 5, vkd = 20, vki = 0;
	//  pid_vx_.setParam(vkp, vki, vkd, 2);
	//  pid_vy_.setParam(vkp, vki, vkd, 2);
	//}
	//构造函数
	PID() {
		pid_vx.reset();
		pid_vy.reset();
		pid_x.reset();
		pid_y.reset();

		pid_vx.setParam(vkp, vki, vkd, 2);
		pid_vy.setParam(vkp, vki, vkd, 2);

		pid_x.setParam(kp, ki, kd, 2);
		pid_y.setParam(kp, ki, kd, 2);
	}
	//析构函数
	~PID() {}
	//成员函数
	double PID::PIDX(double error, double x_max, double tolerance);
	double PID::PIDY(double error, double y_max, double tolerance);
	double PIDXY(double error, double v_max, bool is_X = true);
	//double PID::PIDZ(double reference, double tolerance);
	double PID::PIDZ(double reference, double tolerance, double kp);
	double PID::PIDdis(double reference, double tolerance, uint16_t distance);
	void PIDReset();
private:

	PIDController pid_vx;//速度控制
	PIDController pid_vy;
	PIDController pid_x;//位置控制
	PIDController pid_y;
	/* double lasterrorx_;
	 double lasterrory_;*/
};

extern double kp_g, ki_g, kd_g;//
extern double vkp_g, vki_g, vkd_g;
class PID_GPS {
public:

	PID_GPS() {

		pid_vx.reset();
		pid_vy.reset();
		pid_x.reset();
		pid_y.reset();

		pid_vx.setParam(vkp_g, vki_g, vkd_g, 2);
		pid_vy.setParam(vkp_g, vki_g, vkd_g, 2);

		pid_x.setParam(kp_g, ki_g, kd_g, 2);
		pid_y.setParam(kp_g, ki_g, kd_g, 2);

	}

	~PID_GPS() {}
	double PID_GPS::PIDX(double error, double x_max, double tolerance);
	double PID_GPS::PIDY(double error, double y_max, double tolerance);
	void PID_GPS::PIDReset();
private:

	PIDController pid_vx;//速度控制
	PIDController pid_vy;
	PIDController pid_x;//位置控制
	PIDController pid_y;
	/* double lasterrorx_;
	double lasterrory_;*/
};

//小树林PID
extern double kp_t, ki_t, kd_t;//
extern double vkp_t, vki_t, vkd_t;
class PID_TREE {
public:

	PID_TREE() {

		pid_vx.reset();
		pid_vy.reset();
		pid_x.reset();
		pid_y.reset();

		pid_vx.setParam(vkp_t, vki_t, vkd_t, 2);
		pid_vy.setParam(vkp_t, vki_t, vkd_t, 2);

		pid_x.setParam(kp_t, ki_t, kd_t, 2);
		pid_y.setParam(kp_t, ki_t, kd_t, 2);

	}
	//析构函数
	~PID_TREE() {}
	//成员函数
	double PID_TREE::PIDX(double error, double x_max, double tolerance);
	void PID_TREE::PIDReset();
private:

	PIDController pid_vx;//速度控制
	PIDController pid_vy;
	PIDController pid_x;//位置控制
	PIDController pid_y;
	/* double lasterrorx_;
	double lasterrory_;*/
};
#endif /*PID_H__*/
