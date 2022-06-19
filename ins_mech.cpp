#include<iostream>
#include"ins_mech.h"

using namespace std;


/*******************************************************
* @brief       机械编排姿态更新
* @author      Sihan Wang
* @paras[in]   现在的IMU数据imu_data_2，上一个历元的IMU数据imu_data_1以及机械编排结果result_1
*              IMU的数据是在NED的坐标系下，以增量形式输入
* @paras[out]  现在的机械编排结果result_2，
* @return      void
* @Date        22/5/2022
*******************************************************/
void Attitude_Updating(IMU_NED_DATA* imu_data_1, IMU_NED_DATA* imu_data_2, INS_Result* result_1, INS_Result* result_2)
{
	/********************************** 输入的数据都是增量形式 **********************************/ 

	// 输入现在时刻的加速度数据和陀螺仪数据
	Vector3d delta_Vk(imu_data_2->accel_x, imu_data_2->accel_y, imu_data_2->accel_z);
	Vector3d delta_Thetak(imu_data_2->gyro_x, imu_data_2->gyro_y, imu_data_2->gyro_z);

	// 输入上一个历元时刻的加速度数据和陀螺仪数据
	Vector3d delta_Vk_1(imu_data_1->accel_x, imu_data_1->accel_y, imu_data_1->accel_z);
	Vector3d delta_Thetak_1(imu_data_1->gyro_x, imu_data_1->gyro_y, imu_data_1->gyro_z);


	// 计算当前的子午圈半径和卯酉圈半径
	double RN = cal_RN(result_1->position.B);
	double RM = cal_RM(result_1->position.B);

	// 地球自转角速度
	Vector3d Omegaen_n(result_1->velocity.VE / (RN + result_1->position.H),
		               -result_1->velocity.VN / (RM + result_1->position.H),
		               -result_1->velocity.VE * tan(result_1->position.B) / (RN + result_1->position.H));
	Vector3d Omegaie_n(Omega_WGS * cos(result_1->position.B), 0.0, -Omega_WGS * sin(result_1->position.B));

	Quaterniond q_b;	// Quaternion bk to bk-1
	Quaterniond q_n;	// Quaternion nk-1 to nk

	// 等效旋转矢量
	Vector3d phi_n = (Omegaen_n + Omegaie_n) * IMU_SAMPLE_INTERVAL;
	Vector3d phi = delta_Thetak + 1 / 12.0 * delta_Thetak_1.cross(delta_Thetak);

	// qbb的更新
	double phi_b_norm = phi.norm();
	double wb = cos(0.5 * phi_b_norm);
	double xb = sin(0.5 * phi_b_norm) * 0.5 * phi(0) / (0.5 * phi_b_norm);
	double yb = sin(0.5 * phi_b_norm) * 0.5 * phi(1) / (0.5 * phi_b_norm);
	double zb = sin(0.5 * phi_b_norm) * 0.5 * phi(2) / (0.5 * phi_b_norm);
	q_b = Quaterniond(wb, xb, yb, zb);
	
	// qnn的更新
	double phi_n_norm = phi_n.norm();
	double wn = cos(0.5 * phi_n_norm);
	double xn = -sin(0.5 * phi_n_norm) / (0.5 * phi_n_norm) * 0.5 * phi_n(0);
	double yn = -sin(0.5 * phi_n_norm) / (0.5 * phi_n_norm) * 0.5 * phi_n(1);
	double zn = -sin(0.5 * phi_n_norm) / (0.5 * phi_n_norm) * 0.5 * phi_n(2);
	q_n = Quaterniond(wn, xn, yn, zn);
	
	result_2->attitude.Q = q_n * result_1->attitude.Q * q_b;
	result_2->attitude.Q.normalize();

	Matrix3d rotmat = QuaternionToRotationMatrix(result_2->attitude.Q);
	Matrix3d rotmat_norm = NormolizeR(rotmat);
	Vector3d Euler = RotationMatrixToEuler(rotmat_norm);
	result_2->attitude.E.Roll = Euler(0);
	result_2->attitude.E.Pitch = Euler(1);
	result_2->attitude.E.Yaw = Euler(2);

}

/*******************************************************
* @brief       机械编排速度更新
* @author      Sihan Wang
* @paras[in]   现在的IMU数据imu_data_2，上一个历元的IMU数据imu_data_1, 用于外推的上上一个历元的速度imu_data_0
               以及机械编排结果result_1,用于外推的上上一个机械编排结果result_0
*              IMU的数据是在NED的坐标系下，以增量形式输入
* @paras[out]  现在的机械编排结果result_2，
* @return      void
* @Date        23/5/2022
*******************************************************/
void Velocity_Updating(IMU_NED_DATA* imu_data_0, IMU_NED_DATA* imu_data_1,IMU_NED_DATA* imu_data_2, 
	                   INS_Result* result_0, INS_Result* result_1, INS_Result* result_2)
{
	/********************************** 输入的数据都是增量形式 **********************************/

	Eigen::Vector3d v_fk_n, v_gcor;
	Eigen::Vector3d v_fk_b;
	Eigen::Matrix3d C_bn0, part1;

	double B_half, L_half, H_half;
	double VE_half, VN_half, VD_half;
	B_half = exterpolate(result_1->position.B, result_0->position.B);
	L_half = exterpolate(result_1->position.L, result_0->position.L);
	H_half = exterpolate(result_1->position.H, result_0->position.H);
	VE_half = exterpolate(result_1->velocity.VE, result_0->velocity.VE);
	VN_half = exterpolate(result_1->velocity.VN, result_0->velocity.VN);
	VD_half = exterpolate(result_1->velocity.VD, result_0->velocity.VD);

	double RM, RN;
	RM = cal_RM(result_1->position.B);
	RN = cal_RN(result_1->position.B);

	double RM_half, RN_half;
	RM_half = cal_RM(B_half);
	RN_half = cal_RN(B_half);

	// 中间时刻的牵连角速度
	Vector3d w_en_n(VE_half / (RN_half + H_half),
		           -VN_half / (RM_half + H_half),
				   -VE_half * tan(B_half) / (RN_half + H_half));
	Vector3d w_ie_n(Omega_WGS * cos(B_half), 0, -Omega_WGS * sin(B_half));

	// 计算重力加速度在n系的投影,在某个时刻的纬度lat 和高程h 下
	double g = Cal_Gravity(B_half, H_half);
	Vector3d g_ln (0.0, 0.0, g);
	Vector3d pre_vel(result_1->velocity.VN, result_1->velocity.VE, result_1->velocity.VD);
	Vector3d vel_half(VN_half, VE_half, VD_half);

	//t_k-1 epoch's v,g_ln,w
	v_gcor = (g_ln - ((2 * w_ie_n + w_en_n).cross(vel_half))) * IMU_SAMPLE_INTERVAL;

	// 等效旋转矢量
	Eigen::Vector3d phi = (w_ie_n + w_en_n) * IMU_SAMPLE_INTERVAL;

	part1 = Eigen::Matrix3d::Identity() - 0.5 * SkewMat(phi);

	Vector3d deltavk(imu_data_2->accel_x, imu_data_2->accel_y, imu_data_2->accel_z);
	Vector3d deltathetak(imu_data_2->gyro_x, imu_data_2->gyro_y, imu_data_2->gyro_z);
	Vector3d deltavk_1(imu_data_1->accel_x, imu_data_1->accel_y, imu_data_1->accel_z);
	Vector3d deltathetak_1(imu_data_1->gyro_x, imu_data_1->gyro_y, imu_data_1->gyro_z);

	v_fk_b = deltavk + 0.5 * (deltathetak.cross(deltavk)) + 
		    (deltathetak_1.cross(deltavk) + deltavk_1.cross(deltathetak)) / 12.0;

	// 上一个时刻的Cbn
	C_bn0 = QuaternionToRotationMatrix(result_1->attitude.Q);

	v_fk_n = part1 * C_bn0 * v_fk_b;

	
	Vector3d Velocity_Bef(result_1->velocity.VN, result_1->velocity.VE, result_1->velocity.VD);
	Vector3d Velocity_Cur = Velocity_Bef + v_fk_n + v_gcor;

	result_2->velocity.VN = Velocity_Cur(0);
	result_2->velocity.VE = Velocity_Cur(1);
	result_2->velocity.VD = Velocity_Cur(2);
}

/*******************************************************
* @brief       机械编排位置更新
* @author      Sihan Wang
* @paras[in]   现在的IMU数据imu_data_2，上一个历元的IMU数据imu_data_1以及机械编排结果result_1
*              IMU的数据是在NED的坐标系下，以增量形式输入
* @paras[out]  现在的机械编排结果result_2，
* @return      void
* @Date        23/5/2022
*******************************************************/
void Position_Updating(IMU_NED_DATA* imu_data_1, IMU_NED_DATA* imu_data_2, INS_Result* result_1, INS_Result* result_2)
{
	/********************************** 输入的数据都是增量形式 **********************************/
	double RM, RN;
	RM = cal_RM(result_1->position.B);
	RN = cal_RN(result_1->position.B);

	result_2->position.H = result_1->position.H - 0.5 * (result_1->velocity.VD + result_2->velocity.VD) * IMU_SAMPLE_INTERVAL;
	double h_ave = 0.5 * (result_1->position.H + result_2->position.H);
	double vN_ave = 0.5 * (result_1->velocity.VN + result_2->velocity.VN);
	double vE_ave = 0.5 * (result_1->velocity.VE + result_2->velocity.VE);

	result_2->position.B = result_1->position.B + vN_ave / (RM + h_ave) * IMU_SAMPLE_INTERVAL;

	double lat_ave = 0.5 * (result_1->position.B + result_2->position.B);

	RN = cal_RN(lat_ave);

	result_2->position.L = result_1->position.L + vE_ave / ((RN + h_ave) * cos(lat_ave)) * IMU_SAMPLE_INTERVAL;
}

/*******************************************************
* @brief       机械编排中间时刻参数外推
* @author      Sihan Wang
* @paras[in]   上一个时刻的状态statek_1，上上一个时刻保存的状态statek_2
* @paras[out]  none
* @return      中间时刻的状态
* @Date        23/5/2022
*******************************************************/
double exterpolate(double statek_1, double statek_2)
{
	return 3 / 2 * statek_1 - 1 / 2 * statek_2;
}