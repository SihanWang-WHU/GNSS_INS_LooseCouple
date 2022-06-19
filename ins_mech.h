#pragma once

#ifndef INS_MECH_H_
#define INS_MECH_H_

#include <Eigen/Dense>
#include "Decode_IMR.h"
#include"Config.h"
#include"Data_Functions.h"

// ×ËÌ¬¸üÐÂËã·¨
void Attitude_Updating(IMU_NED_DATA* imu_data_1, IMU_NED_DATA* imu_data_2, INS_Result* result_1, INS_Result* result_2);
void Velocity_Updating(IMU_NED_DATA* imu_data_0, IMU_NED_DATA* imu_data_1, IMU_NED_DATA* imu_data_2,
					   INS_Result* result_0, INS_Result* result_1, INS_Result* result_2);
void Position_Updating(IMU_NED_DATA* imu_data_1, IMU_NED_DATA* imu_data_2, INS_Result* result_1, INS_Result* result_2);
double exterpolate(double statek_1, double statek_2);
#endif
