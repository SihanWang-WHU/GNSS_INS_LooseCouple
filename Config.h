#pragma once
#ifndef _CONFIG_H_
#define _CONFIG_H_
#include"ConstNums.h"

#define   BaseB                  30.528441628       // 基站纬度
#define   BaseL                  114.356976890      // 基站经度
#define   BaseH                  41.2160            // 基站高程


#define   IMU_SAMPLE_HZ          200.0              // 采样频率
#define   IMU_SAMPLE_INTERVAL    1/IMU_SAMPLE_HZ    // 采样间隔
#define   InitEpochs             60000

#define   lever_arm_forward      -0.250             //  m, GNSS to IMU
#define   lever_arm_right        -0.225             //  m, GNSS to IMU
#define   lever_arm_down         -0.995             //  m, GNSS to IMU
//#define   lever_arm_forward      -0.068             //  m, GNSS to IMU
//#define   lever_arm_right        0.2965             //  m, GNSS to IMU
//#define   lever_arm_down         -0.922             //  m, GNSS to IMU

// system noise configuration
#define   sigma_sa               1e-5               // 加速度计动态比例因子标准差
#define   sigma_sg               1e-5               // 陀螺仪动态比例因子标准差
#define   sigma_ba               1.4E-5             // 加速度计动态零偏标准差
#define   sigma_bg               7 * D2R /3600      // 陀螺动态零偏标准差

// 一阶高斯马尔可夫过程相关时间
#define   rt_gb					 3600.0              // 陀螺零偏误差的马尔可夫过程相关时间
#define   rt_ab					 3600.0              // 加速度计零偏误差的马尔可夫过程相关时间
#define   rt_gs					 3600.0              // 陀螺比例因子误差的马尔可夫过程相关时间
#define   rt_as					 3600.0              // 加速度计比例因子误差的马尔可夫过程相关时间
#define   ARW                    0.15 * D2R / 60.0
#define   VRW                    0.57 / 60.0

//init variance configuration
#define   deltapos_flat          4.9e-9             // 经纬度方向位置误差方差
#define   deltapos_vertical      1.0                // 垂直方向位置误差方差
#define   deltavel_flat          0.01               // 水平方向速度误差方差
#define   deltavel_vertical      0.01               // 垂直方向速度误差方差
#define   delta_pitch            1e-6               // 俯仰误差方差
#define   delta_roll             1e-6               // 横滚角误差方差
#define   delta_yaw              0.01               // 航向角误差方差
#define   delta_acc_b            1.6e-6             // 加速度计零偏误差方差
#define   delta_gyro_b           1.6e-5             // 陀螺零偏误差方差
#define   delta_acc_s            1e-6               // 加速度计比例因子误差方差
#define   delta_gyro_s           1e-6               // 陀螺仪比例因子误差方差
#endif