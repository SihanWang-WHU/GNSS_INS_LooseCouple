#pragma once
#ifndef _DATA_FUNCTIONS_H_
#define _DATA_FUNCTIONS_H_

#include"ConstNums.h"
#include"Coordinate.h"
#include"Decode_IMR.h"
#include<Eigen/Dense>

using namespace Eigen;

struct Accelerometer
{
	double f_x = 0.0;
	double f_y = 0.0;
	double f_z = 0.0;
};

struct Gyro
{
	double omega_x = 0.0;
	double omega_y = 0.0;
	double omega_z = 0.0;

};


struct Velocity
{
	double VN;
	double VE;
	double VD;
};

struct Euler
{
	double Roll;
	double Pitch;
	double Yaw;
};

// 用四元数和欧拉角来表示的姿态
// 欧拉角只作直观输出用
struct Attitude
{
	Euler E;
	Quaterniond Q;
};

struct Position
{
	NEU neu;
	double B;
	double L;
	double H;
};

// 增量的IMU与NED坐标统一的数据
struct IMU_NED_DATA
{
	double time = 0.0;              //时间戳

	double accel_x = 0.0;           //x-加速度m/s^2
	double accel_y = 0.0;           //y-加速度m/s^2
	double accel_z = 0.0;           //z-加速度m/s^2

	double gyro_x = 0.0;            //x-角速度rad/s
	double gyro_y = 0.0;            //y-角速度rad/s
	double gyro_z = 0.0;            //z-角速度rad/s
};

struct INS_Result
{
	Velocity velocity;
	Attitude attitude;
	Position position;
};
double Cal_Gravity(double B, double H);

double Deg2Rad(double deg);

double Rad2Deg(double rad);
/*子午圈曲率半径（fai为纬度，单位rad）*/
double cal_RM(double fai);
/*卯酉圈曲率半径（fai为纬度，单位rad）*/
double cal_RN(double fai);
/*(构造反对称阵)*/
Matrix3d SkewMat(Vector3d Vec);
/*LLH到NED坐标系的方向余弦矩阵的填充*/
Matrix3d DR_inv(const Vector3d& llh_pos);
Matrix3d NormolizeR(Eigen::Matrix3d R);
Matrix3d QuaternionToRotationMatrix(const Eigen::Quaterniond& q);
Vector3d RotationMatrixToEuler(const Matrix3d& rot_mat);
Matrix3d EulerToRotationMatrix(const Vector3d& euler);
Quaterniond EulerToQuaternion(const Vector3d& euler);
#endif