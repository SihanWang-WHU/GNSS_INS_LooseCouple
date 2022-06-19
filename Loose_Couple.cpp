#include"Loose_Couple.h"

/*******************************************************
* @brief       松组合卡尔曼滤波代估参数的方差矩阵初始化
* @author      Sihan Wang
* @paras[in]   none
* @paras[out]  none
* @return      void
* @Date        30/5/2022
*******************************************************/
void LCKF::Init_P()
{
	P.setZero();
	P(0, 0) = deltapos_flat;
	P(1, 1) = deltapos_flat;
	P(2, 2) = deltapos_vertical;
	P(3, 3) = deltavel_flat;
	P(4, 4) = deltavel_flat;
	P(5, 5) = deltavel_flat;
	P(6, 6) = delta_roll;
	P(7, 7) = delta_pitch;
	P(8, 8) = delta_yaw;
	P(9, 9) = delta_gyro_b;
	P(10, 10) = delta_gyro_b;
	P(11, 11) = delta_gyro_b;
	P(12, 12) = delta_acc_b; 
	P(13, 13) = delta_acc_b;
	P(14, 14) = delta_acc_b;
	P(15, 15) = delta_gyro_s;
	P(16, 16) = delta_gyro_s;
	P(17, 17) = delta_gyro_s;
	P(18, 18) = delta_acc_s;
	P(19, 19) = delta_acc_s;
	P(20, 20) = delta_acc_s;
}

/*******************************************************
* @brief       松组合卡尔曼滤波功率谱密度矩阵初始化
* @author      Sihan Wang
* @paras[in]   none
* @paras[out]  none
* @return      void
* @Date        30/5/2022
*******************************************************/
void LCKF::Init_Q()
{
	q.setZero();
	q(0, 0) = VRW * VRW;
	q(1, 1) = VRW * VRW;
	q(2, 2) = VRW * VRW;
	q(3, 3) = ARW * ARW;
	q(4, 4) = ARW * ARW;
	q(5, 5) = ARW * ARW;
	q(6, 6) = 2 * sigma_bg * sigma_bg / rt_gb;
	q(7, 7) = 2 * sigma_bg * sigma_bg / rt_gb;
	q(8, 8) = 2 * sigma_bg * sigma_bg / rt_gb;
	q(9, 9) = 2 * sigma_ba * sigma_ba / rt_ab;
	q(10, 10) = 2 * sigma_ba * sigma_ba / rt_ab;
	q(11, 11) = 2 * sigma_ba * sigma_ba / rt_ab;
	q(12, 12) = 2 * sigma_sg * sigma_sg / rt_gs;
	q(13, 13) = 2 * sigma_sg * sigma_sg / rt_gs;
	q(14, 14) = 2 * sigma_sg * sigma_sg / rt_gs;
	q(15, 15) = 2 * sigma_sa * sigma_sa / rt_as;
	q(16, 16) = 2 * sigma_sa * sigma_sa / rt_as;
	q(17, 17) = 2 * sigma_sa * sigma_sa / rt_as;
	Q.setZero();
}

/*******************************************************
* @brief       松组合卡尔曼滤波的构造函数
*              将估计状态的矩阵置零，填充P矩阵以及S矩阵（随参数不变）
* @author      Sihan Wang
* @paras[in]   none
* @paras[out]  none
* @return      void
* @Date        30/5/2022
*******************************************************/
void LCKF::InitKalman()
{
	// 将估计量矩阵置零
	x.setZero();
    // P矩阵填充
	Init_P();
	// S矩阵填充
	Init_Q();
}

/*******************************************************
* @brief       松组合卡尔曼滤波的一步预测过程
*              填充了连续的系数矩阵F并将其离散化
*			   进行了连续时间系统噪声向量的二阶矩矩阵Q和估计状态量的权阵P的似然过程
* @author      Sihan Wang
* @paras[in]   imu_pos     在当地坐标系(LLH)下的上个历元IMU机械编排位置结果
			   imu_vel     在NED坐标系下的上个历元IMU机械编排速度结果
               imu_eul     上个历元IMU机械编排姿态结果(用欧拉角表示)
			   f_bi_b      加速度计的比力输出
			   w_bi_b      陀螺仪的非增量输出
			   dT          传播时间
* @paras[out]  Q           系统噪声矩阵
               P           误差状态方差阵
* @return      void
* @Date        30/5/2022
*******************************************************/
void LCKF::processPredict(const Vector3d& imu_pos, const Vector3d& imu_vel, Vector3d& imu_eul, 
	                      const Vector3d& f_bi_b,  const Vector3d& w_bi_b,  const double dT)
{
	double B = imu_pos(0);
	double L = imu_pos(1);
	double h = imu_pos(2);
	_tor_s = dT;

	double vN = imu_vel(0);
	double vE = imu_vel(1);
	double vD = imu_vel(2);

	double RM = cal_RM(B);
	double RN = cal_RN(B);

	double g = Cal_Gravity(B, h);
	Matrix3d C_b2n;
	C_b2n = EulerToRotationMatrix(imu_eul);

	/***********************F矩阵的填充***************************/
	F.setZero();

	// 位置本身的高阶系数
	// F11
	Matrix3d Frr = Matrix3d::Zero(3, 3);
	Frr(0, 0) = -1.0 * vD / (RM + h);
	Frr(0, 2) = vN / (RM + h);
	Frr(1, 0) = vE * tan(B) / (RN + h);
	Frr(1, 1) = -(vD + vN * tan(B)) / (RN + h);
	Frr(1, 2) = vE / (RN + h);
	F.block<3, 3>(0, 0) = Frr;

	// 位置对于速度的系数
	// F12
	Matrix3d F12 = Matrix3d::Identity();
	F.block<3, 3>(0, 3) = F12;

	// 速度对于位置的系数
	// F21
	Matrix3d Fvr = Matrix3d::Zero(3, 3);
	Fvr(0, 0) = -2 * vE * Omega_WGS * cos(B) / (RM + h) - vE * vE / ((RN + h) * (RM + h) * cos(B) * cos(B));
	Fvr(0, 2) = vN * vD / pow((RM + h), 2) - pow(vE, 2) * tan(B) / pow((RN + h), 2);
	Fvr(1, 0) = 2 * Omega_WGS * (vN * cos(B) - vD * sin(B)) / (RM + h) + vN * vE / ((RN + h) * (RM + h) * pow(cos(B),2));
	Fvr(1, 2) = vE * vD / pow((RN + h), 2) + vN * vE * tan(B) / pow((RN + h), 2);
	Fvr(2, 0) = 2 * vE * Omega_WGS * sin(B) / (RM + h);
	Fvr(2, 2) = - pow(vE, 2) / pow((RN + h), 2) - pow(vN, 2) / pow((RM + h), 2) + 2 * GammaA / (sqrt(RM * RN) + h);
	F.block<3, 3>(3, 0) = Fvr;

	// 速度自身的高阶项
	// F22
	Matrix3d Fvv = Matrix3d::Zero(3, 3);
	Fvv(0, 0) = vD / (RM + h);
	Fvv(0, 1) = -2 * Omega_WGS * sin(B) - 2 * vE * tan(B) / (RN + h);
	Fvv(0, 2) = vN / (RM + h);
	Fvv(1, 0) = 2 * Omega_WGS * sin(B) + vE * tan(B) / (RN + h);
	Fvv(1, 1) = (vD + vN * tan(B)) / (RN + h);
	Fvv(1, 2) = 2 * Omega_WGS * cos(B) + vE / (RN + h);
	Fvv(2, 0) = -2 * vN / (RM + h);
	Fvv(2, 1) = -2 * Omega_WGS * cos(B) - 2 * vE / (RN + h);
	F.block<3, 3>(3, 0) = Fvv;

	// 速度对于姿态的系数
	// F23
	Matrix3d F23 = Matrix3d::Zero(3, 3);
	Vector3d f_bi_n = C_b2n * f_bi_b;
	F23 = SkewMat(f_bi_n);
	F.block<3, 3>(3, 6) = F23;

	// 速度对于加速度计零偏的系数
	// F25
	Matrix3d F25 = C_b2n;
	F.block<3, 3>(3, 12) = F25;

	// 速度对于加速度计比例因子误差的系数
	// 27
	Matrix3d F27 = Matrix3d::Zero(3, 3);
	Matrix3d diagf = f_bi_b.asDiagonal();
	F27 = C_b2n * diagf;
	F.block<3, 3>(3, 18) = F27;

	// 姿态对于位置的系数
	// F31
	Matrix3d Fphir = Matrix3d::Zero(3, 3);
	Fphir(0, 0) = -Omega_WGS * sin(B) / (RM + h);
	Fphir(0, 2) = vE / pow((RN + h), 2);
	Fphir(1, 2) = -vN / pow((RM + h), 2);
	Fphir(2, 0) = -Omega_WGS * cos(B) / (RM + h) - vE / ((RN + h) * (RM + h) * pow(cos(B),2));
	Fphir(2, 2) = -vE * tan(B) / pow((RN + h), 2);
	F.block<3, 3>(6, 0) = Fphir;

	// 姿态对于速度的系数
	// F32
	Matrix3d Fphiv = Matrix3d::Zero(3, 3);
	Fphiv(0, 1) = 1 / (RN + h);
	Fphiv(1, 0) = -1 / (RM + h);
	Fphiv(2, 1) = -tan(B) / (RN + h);
	F.block<3, 3>(6, 3) = Fphiv;

	// 姿态本身的高阶系数
	// F33
	Matrix3d F33 = Matrix3d::Zero(3, 3);
	Vector3d w_ei_n;//地球自转角速度在n系下的投影
	Vector3d w_ne_n;//n系相对于e系的角速度在n系下的投影（牵连角速度）
	Vector3d w_ni_n;//n系相对于i系的角速度在n系下的投影（牵连角速度）
	w_ei_n << Omega_WGS * cos(B), 0, -1.0 * Omega_WGS * sin(B);
	w_ne_n << vE / (RN + h), -1.0 * vN / (RM + h), -1.0 * vE * tan(B) / (RN + h);
	w_ni_n = w_ei_n + w_ne_n;
	F33 = -1.0 * SkewMat(w_ni_n);
	F.block<3, 3>(6, 6) = F33;

	// 姿态对于陀螺仪零偏的系数
	// F34
	Matrix3d F34 = -1 * C_b2n;
	F.block<3, 3>(6, 9) = F34;

	// 姿态对于陀螺仪比例因子误差
	// F36
	Matrix3d F36 = Matrix3d::Zero(3, 3);
	Matrix3d diagw = w_bi_b.asDiagonal();
	F36 = -1 * C_b2n * diagw;
	F.block<3, 3>(6, 15) = F36;

	// 下面是高斯马尔可夫过程
	Matrix3d I33 = Matrix3d::Identity();
	Matrix3d F44 = Matrix3d::Zero(3, 3);
	Matrix3d F55 = Matrix3d::Zero(3, 3);
	Matrix3d F66 = Matrix3d::Zero(3, 3);
	Matrix3d F77 = Matrix3d::Zero(3, 3);
	F44 = -1 / rt_gb * I33;
	F55 = -1 / rt_ab * I33;
	F66 = -1 / rt_gs * I33;
	F77 = -1 / rt_as * I33;
	F.block<3, 3>(9, 9) = F44;
	F.block<3, 3>(12, 12) = F55;
	F.block<3, 3>(15, 15) = F66;
	F.block<3, 3>(18, 18) = F77;

	/***********************G矩阵的填充***************************/
	G.setZero();
	G.block<3, 3>(3, 0) = C_b2n;
	G.block<3, 3>(6, 3) = C_b2n;
	G.block<3, 3>(9, 6) = I33;
	G.block<3, 3>(12, 9) = I33;
	G.block<3, 3>(15, 12) = I33;
	G.block<3, 3>(18, 15) = I33;

	/***********************系统方程***************************/
	// 进行离散化处理
	// Phi矩阵
	// 相关时间短时，可以近似为 I + F * t
	Phi = MatrixXd::Identity(21, 21) + F * _tor_s;

	/***********************一步预测***************************/
	x.setZero();
	
	Q = G * q * G.transpose()*_tor_s;

	// 预测P矩阵
	P = Phi * P * Phi.transpose() + Q;
}


/*******************************************************
* @brief       松组合卡尔曼滤波的量测更新过程
*              计算了观测状态量Z，填充了方差矩阵R
*			   进行了卡尔曼滤波的量测更新过程，计算出这个历元的待估状态
* @author      Sihan Wang
* @paras[in]   imu_pos     在当地坐标系(LLH)下的上个历元IMU机械编排位置结果
			   imu_vel     在NED坐标系下的上个历元IMU机械编排速度结果
			   imu_eul     上个历元IMU机械编排姿态结果(用欧拉角表示)
			   g_bias      陀螺仪零偏
			   a_bias      加速度计零偏
			   gnss_pos    未经过杆臂改正的GNSS的位置观测值
			   gnss_vel    未经过杆臂改正的GNSS的速度观测值
			   Dxx         gnss_pos的方差矩阵
			   Dvv         gnss_vel的方差矩阵
* @paras[out]  imu_pos     经过了滤波更新后的这个历元的位置结果（LLH)
			   imu_vel     经过了滤波更新后的这个历元的速度结果（NED）
			   imu_eul     经过了滤波更新后的这个历元的姿态结果（Euler）
			   g_bias      经过了滤波更新后的这个历元的陀螺仪零偏
			   a_bias      经过了滤波更新后的这个历元的加速度计零偏
* @return      void
* @Date        30/5/2022
*******************************************************/
void LCKF::processUpdate(Vector3d& imu_pos, Vector3d& imu_vel, Vector3d& imu_eul, Vector3d& g_bias, Vector3d& a_bias, 
	                     Vector3d& g_s, Vector3d& a_s, const Vector3d& gnss_pos, const Vector3d& gnss_vel, 
	                     const Matrix3d& Dxx, const Matrix3d& Dvv, Vector3d& Leverarm, Vector3d& w_bi_b)
{
	double B = imu_pos(0);
	double L = imu_pos(1);
	double h = imu_pos(2);

	double vN = imu_vel(0);
	double vE = imu_vel(1);
	double vD = imu_vel(2);

	double RM = cal_RM(B);
	double RN = cal_RN(B);

	Vector3d w_ei_n;//地球自转角速度在n系下的投影
	Vector3d w_ne_n;//n系相对于e系的角速度在n系下的投影（牵连角速度）
	Vector3d w_ni_n;//n系相对于i系的角速度在n系下的投影（牵连角速度）
	w_ei_n << Omega_WGS * cos(B), 0, -1.0 * Omega_WGS * sin(B);
	w_ne_n << vE / (RN + h), -1.0 * vN / (RM + h), -1.0 * vE * tan(B) / (RN + h);
	w_ni_n = w_ei_n + w_ne_n;

	Matrix3d C_b2n;
	C_b2n = EulerToRotationMatrix(imu_eul);

	MatrixXd IFF = MatrixXd::Identity(21, 21);
	Matrix3d I33 = Matrix3d::Identity();

	// DR_inv 将 n 系下的北向、东向和垂向位置差异（单位 m）转化为纬度、经度和高程分量的差异
	Matrix3d DR;
	DR.setZero();
	DR(0, 0) = RM + h;
	DR(1, 1) = (RN + h) * cos(B);
	DR(2, 2) = -1;
	Matrix3d DR_inv = DR.inverse();

	Matrix<double, 3, 18> Hr;
	Matrix<double, 3, 18> Hv;
	Hr.setZero(); Hv.setZero();

	/***********************观测方程***************************/
	Z.setZero();
	Z.block<3, 1>(0, 0) = DR * (imu_pos - gnss_pos) + C_b2n * Leverarm;
	Z.block<3, 1>(3, 0) = imu_vel - gnss_vel - C_b2n * Leverarm.cross(w_bi_b) - SkewMat(w_ni_n) * C_b2n * Leverarm;
	//Z.block<3, 1>(3, 0) = imu_vel - gnss_vel - C_b2n * SkewMat(w_bi_b) * Leverarm - SkewMat(w_ni_n) * C_b2n * Leverarm;

	Matrix<double, 3, 3> Hv3;
	Matrix<double, 3, 3> Hv6;
	H.setZero();
	Hr.block<3, 3>(0, 0) = I33;
	Hr.block<3, 3>(0, 6) = SkewMat(C_b2n * Leverarm);
	Hv3 = -SkewMat(w_ni_n) * SkewMat(C_b2n * Leverarm) - C_b2n * SkewMat(Leverarm.cross(w_bi_b));
	Hv6 = -C_b2n * SkewMat(Leverarm) * w_bi_b.asDiagonal();
	Hv.block<3, 3>(0, 3) = I33;
	Hv.block<3, 3>(0, 6) = Hv3;
	Hv.block<3, 3>(0, 9) = -SkewMat(C_b2n * Leverarm);
	Hv.block<3, 3>(0, 15) = Hv6;
	H.block<3, 18>(0, 0) = Hr;
	H.block<3, 18>(3, 0) = Hv;

	// R矩阵
	R.setZero();
	R.block<3, 3>(0, 0) = Dxx ;
	R.block<3, 3>(3, 3) = Dvv ;

	// 计算出增益矩阵 K
	Matrix<double, 21, 6> K = P * H.transpose() * ((H * P * H.transpose() + R).inverse());
	Matrix<double, 6, 1> V = Z - H * x;

	x = x + K * V;
	P = (IFF - K * H) * P * ((IFF - K * H).transpose()) + K * R * K.transpose();

	Vector3d Pos_error = x.block<3, 1>(0, 0);
	Vector3d _vel_error = x.block<3, 1>(3, 0);
	Vector3d _att_error = x.block<3, 1>(6, 0);
	Vector3d _g_bias_error = x.block<3, 1>(9, 0);
	Vector3d _a_bias_error = x.block<3, 1>(12, 0);
	Vector3d _g_s_error = x.block<3, 1>(15, 0);
	Vector3d _a_s_error = x.block<3, 1>(18, 0);

	// 位置速度的改正
	imu_pos(0) = imu_pos(0) - Pos_error(0) * DR_inv(0, 0);
	imu_pos(1) = imu_pos(1) - Pos_error(1) * DR_inv(1, 1);
	imu_pos(2) = imu_pos(2) - Pos_error(2) * DR_inv(2, 2);
	imu_vel = imu_vel - _vel_error;
	// 姿态的改正
	Matrix3d _C_b2n_new = (I33 - SkewMat(_att_error)).inverse() * C_b2n;
	Matrix3d rotmat_norm = NormolizeR(_C_b2n_new);
	imu_eul = RotationMatrixToEuler(rotmat_norm);
    // 零偏的改正
	g_bias = g_bias + _g_bias_error;
	a_bias = a_bias + _a_bias_error;
	g_s = g_s + _g_s_error;
	a_s = a_s + _a_s_error;
}


/*
correctLeverArm     杆臂改正：（GNSS天线相位中心->INS中心）

gnss_pos    gnss解算纬经高
gnss_vel    gnss解算北东地速度
imu_pos     imu机械编排纬经高
imu_vel     imu机械编排北东地速度
imu_eul     imu机械编排姿态角(roll pitch yaw :rad)
w_bi_b      陀螺三轴输出（rad/s）
lever_arm   b系下的三轴杆臂值
*/
void correctLeverArm(Vector3d& gnss_pos, Vector3d& gnss_vel, const Vector3d& imu_pos, const Vector3d& imu_vel, const Vector3d& imu_eul, const Vector3d& w_bi_b, const Vector3d& lever_arm)
{
	double lat = imu_pos(0);
	double lon = imu_pos(1);
	double height = imu_pos(2);
	double vN = imu_vel(0);
	double vE = imu_vel(1);
	double vD = imu_vel(2);
	double RM = cal_RM(lat);//子午圈曲率半径
	double RN = cal_RN(lat);//卯酉圈曲率半径

	Vector3d eul = imu_eul;//获取姿态角
	Vector3d w_ei_n;//地球自转角速度在n系下的投影
	Vector3d w_ne_n;//n系相对于e系的角速度在n系下的投影（牵连角速度）
	Vector3d w_ni_n;//n系相对于i系的角速度在n系下的投影（牵连角速度）

	w_ei_n << Omega_WGS * cos(lat), 0, -1.0 * Omega_WGS * sin(lat);
	w_ne_n << vE / (RN + height), -1.0 * vN / (RM + height), -1.0 * vE * tan(lat) / (RN + height);
	w_ni_n = w_ei_n + w_ne_n;

	Matrix3d mat_n2l = Matrix3d::Zero(3, 3);//将北东地转化为LLH系
	mat_n2l(0, 0) = 1.0 / (RM + height);
	mat_n2l(1, 1) = 1.0 / ((RN + height) * cos(lat));
	mat_n2l(2, 2) = -1.0;
	Matrix3d C_b2n = Matrix3d::Zero(3, 3);//b系到n系的
	C_b2n = EulerToRotationMatrix(imu_eul);

	Matrix3d OMIGA_ni_n = SkewMat(w_ni_n);
	Matrix3d OMIGA_bi_b = SkewMat(w_bi_b);
	/*位置改正*/
	gnss_pos = gnss_pos - mat_n2l * C_b2n * lever_arm;
	/*速度改正*/
	gnss_vel = gnss_vel + C_b2n * OMIGA_bi_b * lever_arm + OMIGA_ni_n * C_b2n * lever_arm;
}

// 参考椭球体表面的地心半径为
double ReS(double fai)
{
	double RN = cal_RN(fai);
	double ReS = RN * sqrt(cos(fai) * cos(fai) + (1 - E2_WGS84) * (1 - E2_WGS84) * sin(fai) * sin(fai));
	return ReS;
}