#include"Data_Functions.h"

/*******************************************************
* @brief       ���㵱ǰγ�Ⱥ͸߳��µ���������
* @author      Sihan Wang
* @paras[in]   B��rad����H
* @paras[out]  none
* @return      Gravity
* @Date        20/5/2022
*******************************************************/
double Cal_Gravity(double B, double H)
{
	double Gamma_Phi = (R_WGS84 * GammaA * cos(B) * cos(B) + B_WGS84 * GammaB * sin(B) * sin(B))
		/ sqrt(R_WGS84 * R_WGS84 * cos(B) * cos(B) + B_WGS84 * B_WGS84 * sin(B) * sin(B));

	double m = Omega_WGS * Omega_WGS * R_WGS84 * R_WGS84 * B_WGS84 / GM_JGM3;
	double Gravity = Gamma_Phi * (1 - 2 / R_WGS84 * (1 + F_WGS84 + m - 2 * F_WGS84 * sin(B) * sin(B)) * H + 3 / (R_WGS84 * R_WGS84) * H * H);
	return Gravity;
}


/*******************************************************
* @brief       ����î��Ȧ���ʰ뾶
* @author      Sihan Wang
* @paras[in]   fai��γ�ȣ�rad��
* @paras[out]  none
* @return      RN
* @Date        20/5/2022
*******************************************************/
double cal_RN(double fai)
{
	double W = sqrt(1.0 - E2_WGS84 * sin(fai) * sin(fai));
	double RN = R_WGS84 / W;
	return RN;
}

/*******************************************************
* @brief       ��������Ȧ���ʰ뾶
* @author      Sihan Wang
* @paras[in]   fai��γ�ȣ�rad��
* @paras[out]  none
* @return      RM
* @Date        20/5/2022
*******************************************************/
double cal_RM(double fai)
{
	double W = sqrt(1.0 - E2_WGS84 * sin(fai) * sin(fai));
	double RM = R_WGS84 * (1.0 - E2_WGS84) / (W * W * W);
	return RM;
}

/*******************************************************
* @brief       ������ת�Ƕ���
* @author      Sihan Wang
* @paras[in]   rad
* @paras[out]  none
* @return      deg
* @Date        20/5/2022
*******************************************************/
double Rad2Deg(double rad) {
	return rad * 180.0 / PI;
}

/*******************************************************
* @brief       �Ƕ���ת������
* @author      Sihan Wang
* @paras[in]   deg
* @paras[out]  none
* @return      rad
* @Date        20/5/2022
*******************************************************/
double Deg2Rad(double deg) {
	return (deg * PI) / 180.0;
}

/*******************************************************
* @brief       ���Գƾ���
* @author      Sihan Wang
* @paras[in]   ��ά����Vec
* @paras[out]  none
* @return      ���Գƾ���Skew
* @Date        20/5/2022
*******************************************************/
Matrix3d SkewMat(Vector3d Vec)
{
	Matrix3d Skew = Matrix3d::Zero(3, 3);
	Skew(0, 1) = -Vec(2); Skew(0, 2) = Vec(1);
	Skew(1, 0) = Vec(2); Skew(1, 2) = -Vec(0);
	Skew(2, 0) = -Vec(1); Skew(2, 1) = Vec(0);
	return Skew;
}

/*******************************************************
* @brief       �������Ҿ���ת����ŷ����
* @author      Sihan Wang
* @paras[in]   ��ά���������Ҿ���
* @paras[out]  none
* @return      ��ά������ʾ��ŷ���ǣ��ֱ���roll������� pitch�������� yaw������
* @Date        20/5/2022
*******************************************************/
Vector3d RotationMatrixToEuler(const Matrix3d& rot_mat)
{
	Vector3d euler_angle;
	euler_angle(0) = atan2(rot_mat(2, 1), rot_mat(2, 2));
	euler_angle(1) = atan2(-rot_mat(2, 0), sqrt(rot_mat(2, 1) * rot_mat(2, 1) + rot_mat(2, 2) * rot_mat(2, 2)));
	euler_angle(2) = atan2(rot_mat(1, 0), rot_mat(0, 0));
	return euler_angle;
}

/*******************************************************
* @brief       ŷ����ת�����������Ҿ���
* @author      Sihan Wang
* @paras[in]   ��ά������ʾ��ŷ���ǣ��ֱ���roll������� pitch�������� yaw������
*              ŷ���ǵĵ�λ�ǻ���
* @paras[out]  none
* @return      �������Ҿ���
* @Date        20/5/2022
*******************************************************/
Matrix3d EulerToRotationMatrix(const Vector3d& euler)
{
	double Pitch, Roll, Yaw;
	Roll = euler(0);  Pitch = euler(1);  Yaw = euler(2);

	Matrix3d rot_mat = Matrix3d::Zero();
	rot_mat(0, 0) = cos(Pitch) * cos(Yaw);
	rot_mat(0, 1) = - cos(Roll) * sin(Yaw)
		            + sin(Roll) * sin(Pitch) * cos(Yaw);
	rot_mat(0, 2) = sin(Roll) * sin(Yaw)
		            + cos(Roll) * sin(Pitch) * cos(Yaw);
	rot_mat(1, 0) = cos(Pitch) * sin(Yaw);
	rot_mat(1, 1) = cos(Roll) * cos(Yaw)
		            + sin(Roll) * sin(Pitch) * sin(Yaw);
	rot_mat(1, 2) = - sin(Roll) * cos(Yaw)
		            + cos(Roll) * sin(Pitch) * sin(Yaw);
	rot_mat(2, 0) = - sin(Pitch);
	rot_mat(2, 1) = sin(Roll) * cos(Pitch);
	rot_mat(2, 2) = cos(Roll) * cos(Pitch);
	return rot_mat;
}

/*******************************************************
* @brief       ŷ����ת������Ԫ��
* @author      Sihan Wang
* @paras[in]   ��ά������ʾ��ŷ���ǣ��ֱ���roll������� pitch�������� yaw������
* @paras[out]  none
* @return      ��Ԫ��
* @Date        20/5/2022
*******************************************************/
Quaterniond EulerToQuaternion(const Vector3d& euler)
{
	double half_phi = euler(0) / 2.0;
	double half_theta = euler(1) / 2.0;
	double half_psi = euler(2) / 2.0;

	//c1 means cos(halfPhi), c2 means cos(halfTheta), c3 means cos(halfPsi)
	//s1 means sin(halfPhi), s2 means sin(halfTheta), s3 means sin(halfPsi)
	double s1 = sin(half_phi);
	double s2 = sin(half_theta);
	double s3 = sin(half_psi);
	double c1 = cos(half_phi);
	double c2 = cos(half_theta);
	double c3 = cos(half_psi);

	Quaterniond q;
	q.w() = c1 * c2 * c3 + s1 * s2 * s3;
	q.x() = s1 * c2 * c3 - c1 * s2 * s3;
	q.y() = c1 * s2 * c3 + s1 * c2 * s3;
	q.z() = c1 * c2 * s3 - s1 * s2 * c3;

	return q;
}

/*******************************************************
* @brief       ��Ԫ��ת�����������Ҿ���
* @author      Sihan Wang
* @paras[in]   ��Ԫ��q
* @paras[out]  none
* @return      �������Ҿ���
* @Date        21/5/2022
*******************************************************/
Matrix3d QuaternionToRotationMatrix(const Eigen::Quaterniond& q)
{
	double q0 = q.w();
	double q1 = q.x();
	double q2 = q.y();
	double q3 = q.z();

	double c11 = q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;
	double c12 = 2 * (q1 * q2 - q0 * q3);
	double c13 = 2 * (q1 * q3 + q0 * q2);
	double c21 = 2 * (q1 * q2 + q0 * q3);
	double c22 = q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3;
	double c23 = 2 * (q2 * q3 - q0 * q1);
	double c31 = 2 * (q1 * q3 - q0 * q2);
	double c32 = 2 * (q2 * q3 + q0 * q1);
	double c33 = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

	Eigen::Matrix3d DCM;
	DCM << c11, c12, c13,
		c21, c22, c23,
		c31, c32, c33;
	return DCM;
}

/*******************************************************
* @brief       �����NEDϵת����LLH(γ���ߣ�����ϵ�ķ������Ҿ���
* @author      Sihan Wang
* @paras[in]   LLHϵ�µ�����
* @paras[out]  none
* @return      �������Ҿ���
* @Date        26/5/2022
*******************************************************/
Matrix3d DR_inv(const Vector3d& llh_pos)
{
	double lat    = llh_pos(0);
	double lon    = llh_pos(1);
	double height = llh_pos(2);
	Matrix3d mat_n2l = Matrix3d::Zero(3, 3);//��������ת��ΪLLHϵ
	mat_n2l(0, 0) = 1.0 / (cal_RM(lat) + height);
	mat_n2l(1, 1) = 1.0 / ((cal_RN(lat) + height) * cos(lat));
	mat_n2l(2, 2) = -1.0;
	return mat_n2l;
}

/*******************************************************
* @brief       ��ת����������
* @author      Sihan Wang
* @paras[in]   ��Ҫ�������ľ���
* @paras[out]  none
* @return      ������֮��ľ���
* @Date        26/5/2022
*******************************************************/
Matrix3d NormolizeR(Matrix3d R)
{

	Vector3d x = R.col(0);
	Vector3d y = R.col(1);
	double error = x.dot(y);
	Vector3d x_ort = x - y * (error / 2.0);
	Vector3d y_ort = y - x * (error / 2.0);
	Vector3d z_ort = x_ort.cross(y_ort);
	Matrix3d result;
	result.col(0) = 0.5 * (3 - x_ort.dot(x_ort)) * x_ort;
	result.col(1) = 0.5 * (3 - y_ort.dot(y_ort)) * y_ort;
	result.col(2) = 0.5 * (3 - z_ort.dot(z_ort)) * z_ort;

	return result;
}