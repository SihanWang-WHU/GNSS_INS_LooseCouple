#include"Data_Functions.h"

/*******************************************************
* @brief       计算当前纬度和高程下的正常重力
* @author      Sihan Wang
* @paras[in]   B（rad），H
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
* @brief       计算卯酉圈曲率半径
* @author      Sihan Wang
* @paras[in]   fai（纬度，rad）
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
* @brief       计算子午圈曲率半径
* @author      Sihan Wang
* @paras[in]   fai（纬度，rad）
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
* @brief       弧度制转角度制
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
* @brief       角度制转弧度制
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
* @brief       反对称矩阵
* @author      Sihan Wang
* @paras[in]   三维向量Vec
* @paras[out]  none
* @return      反对称矩阵Skew
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
* @brief       方向余弦矩阵转换到欧拉角
* @author      Sihan Wang
* @paras[in]   三维矩阵方向余弦矩阵
* @paras[out]  none
* @return      三维向量表示的欧拉角，分别是roll（横滚） pitch（俯仰） yaw（航向）
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
* @brief       欧拉角转换到方向余弦矩阵
* @author      Sihan Wang
* @paras[in]   三维向量表示的欧拉角，分别是roll（横滚） pitch（俯仰） yaw（航向）
*              欧拉角的单位是弧度
* @paras[out]  none
* @return      方向余弦矩阵
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
* @brief       欧拉角转换到四元数
* @author      Sihan Wang
* @paras[in]   三维向量表示的欧拉角，分别是roll（横滚） pitch（俯仰） yaw（航向）
* @paras[out]  none
* @return      四元数
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
* @brief       四元数转换到方向余弦矩阵
* @author      Sihan Wang
* @paras[in]   四元数q
* @paras[out]  none
* @return      方向余弦矩阵
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
* @brief       计算从NED系转换到LLH(纬经高）坐标系的方向余弦矩阵
* @author      Sihan Wang
* @paras[in]   LLH系下的坐标
* @paras[out]  none
* @return      方向余弦矩阵
* @Date        26/5/2022
*******************************************************/
Matrix3d DR_inv(const Vector3d& llh_pos)
{
	double lat    = llh_pos(0);
	double lon    = llh_pos(1);
	double height = llh_pos(2);
	Matrix3d mat_n2l = Matrix3d::Zero(3, 3);//将北东地转化为LLH系
	mat_n2l(0, 0) = 1.0 / (cal_RM(lat) + height);
	mat_n2l(1, 1) = 1.0 / ((cal_RN(lat) + height) * cos(lat));
	mat_n2l(2, 2) = -1.0;
	return mat_n2l;
}

/*******************************************************
* @brief       旋转矩阵正交化
* @author      Sihan Wang
* @paras[in]   需要正交化的矩阵
* @paras[out]  none
* @return      正交化之后的矩阵
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