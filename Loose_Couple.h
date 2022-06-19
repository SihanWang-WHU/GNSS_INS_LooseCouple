#pragma once
#ifndef _LOOSE_COUPLED_
#define _LOOSE_COUPLED_	
#include"Config.h"
#include"Data_Functions.h"
#include"Decode_GNSS.h"
#include"Eigen/Dense"

using namespace std;
using namespace Eigen;

class LCKF
{
private:
	//double _g_noise_psd;         // gyro measurement noise psd(3 axis independent and have equal variance)
	//double _a_noise_psd;         // accel measurement noise psd(3 axis independent and have equal variance)
	////gyro and accel bias Parameters: modeled as  white moise
	//double _bgd_psd;             // gyro dynamic bias psd(3 axis independent and have equal variance)
	//double _bad_psd;             // accel dynamic bias psd(3 axis independent and have equal variance)
	Matrix<double, 21, 21> F;    // F: differential equation coefficient matrix
	Matrix<double, 21, 18> G;    // G: ����ϵͳ����ϵ������

public:
	//ϵ������
	double _tor_s;//propagation interval(s)
	Matrix<double, 21, 1>  x;    // x����: ��λ�ã��ٶȣ���̬��������ƫ�����ٶȼ���ƫ�������Ǳ������������ٶȼƱ���������
	Matrix<double, 21, 21> Phi;  // Phi: ״̬ת�ƾ���
	Matrix<double, 6, 1>   Z;    // Z: �۲����
	Matrix<double, 6, 21>  H;    // H: �۲�ֵ��ϵ������
	// �������
	Matrix<double, 18, 18> q;
	Matrix<double, 21, 21> Q;    // Q: ϵͳ�����������
	Matrix<double, 21, 21> P;    // P: ���������ķ������
	Matrix<double, 6, 6>   R;    // R: �۲�ֵ�ķ������

public:
	// ����Ͽ������˲��Ĺ��캯��
	void InitKalman();

	void processPredict(const Vector3d& imu_pos, const Vector3d& imu_vel, Vector3d& imu_eul,
						const Vector3d& f_bi_b,  const Vector3d& w_bi_b, const double dT);

	void processUpdate(Vector3d& imu_pos, Vector3d& imu_vel, Vector3d& imu_eul, Vector3d& g_bias, Vector3d& a_bias, Vector3d& g_s, Vector3d& a_s,
		               const Vector3d& gnss_pos, const Vector3d& gnss_vel, const Matrix3d& Dxx, const Matrix3d& Dvv, 
					   Vector3d& Leverarm, Vector3d& w_bi_b);
private:
	void Init_P();// 
	void Init_Q();//initial S
};

double ReS(double fai);
// �˱۸���
void correctLeverArm(Vector3d& gnss_pos, Vector3d& gnss_vel, const Vector3d& imu_pos, const Vector3d& imu_vel, const Vector3d& imu_eul, const Vector3d& w_bi_b, const Vector3d& lever_arm);
#endif