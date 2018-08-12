#pragma once

//airsim
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "api/RpcLibClientBase.hpp"


//传感器数据
extern msr::airlib::GpsData		 GPS_data;
extern msr::airlib::BarometerData	 Barometer_data;
extern msr::airlib::MagnetometerData Magnetometer_data;
extern msr::airlib::ImuData			 Imu_data;


