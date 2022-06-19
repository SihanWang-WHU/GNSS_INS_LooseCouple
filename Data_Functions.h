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

// ����Ԫ����ŷ��������ʾ����̬
// ŷ����ֻ��ֱ�������
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

// ������IMU��NED����ͳһ������
struct IMU_NED_DATA
{
	double time = 0.0;              //ʱ���

	double accel_x = 0.0;           //x-���ٶ�m/s^2
	double accel_y = 0.0;           //y-���ٶ�m/s^2
	double accel_z = 0.0;           //z-���ٶ�m/s^2

	double gyro_x = 0.0;            //x-���ٶ�rad/s
	double gyro_y = 0.0;            //y-���ٶ�rad/s
	double gyro_z = 0.0;            //z-���ٶ�rad/s
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
/*����Ȧ���ʰ뾶��faiΪγ�ȣ���λrad��*/
double cal_RM(double fai);
/*î��Ȧ���ʰ뾶��faiΪγ�ȣ���λrad��*/
double cal_RN(double fai);
/*(���췴�Գ���)*/
Matrix3d SkewMat(Vector3d Vec);
/*LLH��NED����ϵ�ķ������Ҿ�������*/
Matrix3d DR_inv(const Vector3d& llh_pos);
Matrix3d NormolizeR(Eigen::Matrix3d R);
Matrix3d QuaternionToRotationMatrix(const Eigen::Quaterniond& q);
Vector3d RotationMatrixToEuler(const Matrix3d& rot_mat);
Matrix3d EulerToRotationMatrix(const Vector3d& euler);
Quaterniond EulerToQuaternion(const Vector3d& euler);
#endif