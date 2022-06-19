#include"Loose_Couple.h"

/*******************************************************
* @brief       ����Ͽ������˲����������ķ�������ʼ��
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
* @brief       ����Ͽ������˲��������ܶȾ����ʼ��
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
* @brief       ����Ͽ������˲��Ĺ��캯��
*              ������״̬�ľ������㣬���P�����Լ�S������������䣩
* @author      Sihan Wang
* @paras[in]   none
* @paras[out]  none
* @return      void
* @Date        30/5/2022
*******************************************************/
void LCKF::InitKalman()
{
	// ����������������
	x.setZero();
    // P�������
	Init_P();
	// S�������
	Init_Q();
}

/*******************************************************
* @brief       ����Ͽ������˲���һ��Ԥ�����
*              �����������ϵ������F��������ɢ��
*			   ����������ʱ��ϵͳ���������Ķ��׾ؾ���Q�͹���״̬����Ȩ��P����Ȼ����
* @author      Sihan Wang
* @paras[in]   imu_pos     �ڵ�������ϵ(LLH)�µ��ϸ���ԪIMU��е����λ�ý��
			   imu_vel     ��NED����ϵ�µ��ϸ���ԪIMU��е�����ٶȽ��
               imu_eul     �ϸ���ԪIMU��е������̬���(��ŷ���Ǳ�ʾ)
			   f_bi_b      ���ٶȼƵı������
			   w_bi_b      �����ǵķ��������
			   dT          ����ʱ��
* @paras[out]  Q           ϵͳ��������
               P           ���״̬������
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

	/***********************F��������***************************/
	F.setZero();

	// λ�ñ���ĸ߽�ϵ��
	// F11
	Matrix3d Frr = Matrix3d::Zero(3, 3);
	Frr(0, 0) = -1.0 * vD / (RM + h);
	Frr(0, 2) = vN / (RM + h);
	Frr(1, 0) = vE * tan(B) / (RN + h);
	Frr(1, 1) = -(vD + vN * tan(B)) / (RN + h);
	Frr(1, 2) = vE / (RN + h);
	F.block<3, 3>(0, 0) = Frr;

	// λ�ö����ٶȵ�ϵ��
	// F12
	Matrix3d F12 = Matrix3d::Identity();
	F.block<3, 3>(0, 3) = F12;

	// �ٶȶ���λ�õ�ϵ��
	// F21
	Matrix3d Fvr = Matrix3d::Zero(3, 3);
	Fvr(0, 0) = -2 * vE * Omega_WGS * cos(B) / (RM + h) - vE * vE / ((RN + h) * (RM + h) * cos(B) * cos(B));
	Fvr(0, 2) = vN * vD / pow((RM + h), 2) - pow(vE, 2) * tan(B) / pow((RN + h), 2);
	Fvr(1, 0) = 2 * Omega_WGS * (vN * cos(B) - vD * sin(B)) / (RM + h) + vN * vE / ((RN + h) * (RM + h) * pow(cos(B),2));
	Fvr(1, 2) = vE * vD / pow((RN + h), 2) + vN * vE * tan(B) / pow((RN + h), 2);
	Fvr(2, 0) = 2 * vE * Omega_WGS * sin(B) / (RM + h);
	Fvr(2, 2) = - pow(vE, 2) / pow((RN + h), 2) - pow(vN, 2) / pow((RM + h), 2) + 2 * GammaA / (sqrt(RM * RN) + h);
	F.block<3, 3>(3, 0) = Fvr;

	// �ٶ�����ĸ߽���
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

	// �ٶȶ�����̬��ϵ��
	// F23
	Matrix3d F23 = Matrix3d::Zero(3, 3);
	Vector3d f_bi_n = C_b2n * f_bi_b;
	F23 = SkewMat(f_bi_n);
	F.block<3, 3>(3, 6) = F23;

	// �ٶȶ��ڼ��ٶȼ���ƫ��ϵ��
	// F25
	Matrix3d F25 = C_b2n;
	F.block<3, 3>(3, 12) = F25;

	// �ٶȶ��ڼ��ٶȼƱ�����������ϵ��
	// 27
	Matrix3d F27 = Matrix3d::Zero(3, 3);
	Matrix3d diagf = f_bi_b.asDiagonal();
	F27 = C_b2n * diagf;
	F.block<3, 3>(3, 18) = F27;

	// ��̬����λ�õ�ϵ��
	// F31
	Matrix3d Fphir = Matrix3d::Zero(3, 3);
	Fphir(0, 0) = -Omega_WGS * sin(B) / (RM + h);
	Fphir(0, 2) = vE / pow((RN + h), 2);
	Fphir(1, 2) = -vN / pow((RM + h), 2);
	Fphir(2, 0) = -Omega_WGS * cos(B) / (RM + h) - vE / ((RN + h) * (RM + h) * pow(cos(B),2));
	Fphir(2, 2) = -vE * tan(B) / pow((RN + h), 2);
	F.block<3, 3>(6, 0) = Fphir;

	// ��̬�����ٶȵ�ϵ��
	// F32
	Matrix3d Fphiv = Matrix3d::Zero(3, 3);
	Fphiv(0, 1) = 1 / (RN + h);
	Fphiv(1, 0) = -1 / (RM + h);
	Fphiv(2, 1) = -tan(B) / (RN + h);
	F.block<3, 3>(6, 3) = Fphiv;

	// ��̬����ĸ߽�ϵ��
	// F33
	Matrix3d F33 = Matrix3d::Zero(3, 3);
	Vector3d w_ei_n;//������ת���ٶ���nϵ�µ�ͶӰ
	Vector3d w_ne_n;//nϵ�����eϵ�Ľ��ٶ���nϵ�µ�ͶӰ��ǣ�����ٶȣ�
	Vector3d w_ni_n;//nϵ�����iϵ�Ľ��ٶ���nϵ�µ�ͶӰ��ǣ�����ٶȣ�
	w_ei_n << Omega_WGS * cos(B), 0, -1.0 * Omega_WGS * sin(B);
	w_ne_n << vE / (RN + h), -1.0 * vN / (RM + h), -1.0 * vE * tan(B) / (RN + h);
	w_ni_n = w_ei_n + w_ne_n;
	F33 = -1.0 * SkewMat(w_ni_n);
	F.block<3, 3>(6, 6) = F33;

	// ��̬������������ƫ��ϵ��
	// F34
	Matrix3d F34 = -1 * C_b2n;
	F.block<3, 3>(6, 9) = F34;

	// ��̬���������Ǳ����������
	// F36
	Matrix3d F36 = Matrix3d::Zero(3, 3);
	Matrix3d diagw = w_bi_b.asDiagonal();
	F36 = -1 * C_b2n * diagw;
	F.block<3, 3>(6, 15) = F36;

	// �����Ǹ�˹����ɷ����
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

	/***********************G��������***************************/
	G.setZero();
	G.block<3, 3>(3, 0) = C_b2n;
	G.block<3, 3>(6, 3) = C_b2n;
	G.block<3, 3>(9, 6) = I33;
	G.block<3, 3>(12, 9) = I33;
	G.block<3, 3>(15, 12) = I33;
	G.block<3, 3>(18, 15) = I33;

	/***********************ϵͳ����***************************/
	// ������ɢ������
	// Phi����
	// ���ʱ���ʱ�����Խ���Ϊ I + F * t
	Phi = MatrixXd::Identity(21, 21) + F * _tor_s;

	/***********************һ��Ԥ��***************************/
	x.setZero();
	
	Q = G * q * G.transpose()*_tor_s;

	// Ԥ��P����
	P = Phi * P * Phi.transpose() + Q;
}


/*******************************************************
* @brief       ����Ͽ������˲���������¹���
*              �����˹۲�״̬��Z������˷������R
*			   �����˿������˲���������¹��̣�����������Ԫ�Ĵ���״̬
* @author      Sihan Wang
* @paras[in]   imu_pos     �ڵ�������ϵ(LLH)�µ��ϸ���ԪIMU��е����λ�ý��
			   imu_vel     ��NED����ϵ�µ��ϸ���ԪIMU��е�����ٶȽ��
			   imu_eul     �ϸ���ԪIMU��е������̬���(��ŷ���Ǳ�ʾ)
			   g_bias      ��������ƫ
			   a_bias      ���ٶȼ���ƫ
			   gnss_pos    δ�����˱۸�����GNSS��λ�ù۲�ֵ
			   gnss_vel    δ�����˱۸�����GNSS���ٶȹ۲�ֵ
			   Dxx         gnss_pos�ķ������
			   Dvv         gnss_vel�ķ������
* @paras[out]  imu_pos     �������˲����º�������Ԫ��λ�ý����LLH)
			   imu_vel     �������˲����º�������Ԫ���ٶȽ����NED��
			   imu_eul     �������˲����º�������Ԫ����̬�����Euler��
			   g_bias      �������˲����º�������Ԫ����������ƫ
			   a_bias      �������˲����º�������Ԫ�ļ��ٶȼ���ƫ
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

	Vector3d w_ei_n;//������ת���ٶ���nϵ�µ�ͶӰ
	Vector3d w_ne_n;//nϵ�����eϵ�Ľ��ٶ���nϵ�µ�ͶӰ��ǣ�����ٶȣ�
	Vector3d w_ni_n;//nϵ�����iϵ�Ľ��ٶ���nϵ�µ�ͶӰ��ǣ�����ٶȣ�
	w_ei_n << Omega_WGS * cos(B), 0, -1.0 * Omega_WGS * sin(B);
	w_ne_n << vE / (RN + h), -1.0 * vN / (RM + h), -1.0 * vE * tan(B) / (RN + h);
	w_ni_n = w_ei_n + w_ne_n;

	Matrix3d C_b2n;
	C_b2n = EulerToRotationMatrix(imu_eul);

	MatrixXd IFF = MatrixXd::Identity(21, 21);
	Matrix3d I33 = Matrix3d::Identity();

	// DR_inv �� n ϵ�µı��򡢶���ʹ���λ�ò��죨��λ m��ת��Ϊγ�ȡ����Ⱥ͸̷߳����Ĳ���
	Matrix3d DR;
	DR.setZero();
	DR(0, 0) = RM + h;
	DR(1, 1) = (RN + h) * cos(B);
	DR(2, 2) = -1;
	Matrix3d DR_inv = DR.inverse();

	Matrix<double, 3, 18> Hr;
	Matrix<double, 3, 18> Hv;
	Hr.setZero(); Hv.setZero();

	/***********************�۲ⷽ��***************************/
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

	// R����
	R.setZero();
	R.block<3, 3>(0, 0) = Dxx ;
	R.block<3, 3>(3, 3) = Dvv ;

	// ������������ K
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

	// λ���ٶȵĸ���
	imu_pos(0) = imu_pos(0) - Pos_error(0) * DR_inv(0, 0);
	imu_pos(1) = imu_pos(1) - Pos_error(1) * DR_inv(1, 1);
	imu_pos(2) = imu_pos(2) - Pos_error(2) * DR_inv(2, 2);
	imu_vel = imu_vel - _vel_error;
	// ��̬�ĸ���
	Matrix3d _C_b2n_new = (I33 - SkewMat(_att_error)).inverse() * C_b2n;
	Matrix3d rotmat_norm = NormolizeR(_C_b2n_new);
	imu_eul = RotationMatrixToEuler(rotmat_norm);
    // ��ƫ�ĸ���
	g_bias = g_bias + _g_bias_error;
	a_bias = a_bias + _a_bias_error;
	g_s = g_s + _g_s_error;
	a_s = a_s + _a_s_error;
}


/*
correctLeverArm     �˱۸�������GNSS������λ����->INS���ģ�

gnss_pos    gnss����γ����
gnss_vel    gnss���㱱�����ٶ�
imu_pos     imu��е����γ����
imu_vel     imu��е���ű������ٶ�
imu_eul     imu��е������̬��(roll pitch yaw :rad)
w_bi_b      �������������rad/s��
lever_arm   bϵ�µ�����˱�ֵ
*/
void correctLeverArm(Vector3d& gnss_pos, Vector3d& gnss_vel, const Vector3d& imu_pos, const Vector3d& imu_vel, const Vector3d& imu_eul, const Vector3d& w_bi_b, const Vector3d& lever_arm)
{
	double lat = imu_pos(0);
	double lon = imu_pos(1);
	double height = imu_pos(2);
	double vN = imu_vel(0);
	double vE = imu_vel(1);
	double vD = imu_vel(2);
	double RM = cal_RM(lat);//����Ȧ���ʰ뾶
	double RN = cal_RN(lat);//î��Ȧ���ʰ뾶

	Vector3d eul = imu_eul;//��ȡ��̬��
	Vector3d w_ei_n;//������ת���ٶ���nϵ�µ�ͶӰ
	Vector3d w_ne_n;//nϵ�����eϵ�Ľ��ٶ���nϵ�µ�ͶӰ��ǣ�����ٶȣ�
	Vector3d w_ni_n;//nϵ�����iϵ�Ľ��ٶ���nϵ�µ�ͶӰ��ǣ�����ٶȣ�

	w_ei_n << Omega_WGS * cos(lat), 0, -1.0 * Omega_WGS * sin(lat);
	w_ne_n << vE / (RN + height), -1.0 * vN / (RM + height), -1.0 * vE * tan(lat) / (RN + height);
	w_ni_n = w_ei_n + w_ne_n;

	Matrix3d mat_n2l = Matrix3d::Zero(3, 3);//��������ת��ΪLLHϵ
	mat_n2l(0, 0) = 1.0 / (RM + height);
	mat_n2l(1, 1) = 1.0 / ((RN + height) * cos(lat));
	mat_n2l(2, 2) = -1.0;
	Matrix3d C_b2n = Matrix3d::Zero(3, 3);//bϵ��nϵ��
	C_b2n = EulerToRotationMatrix(imu_eul);

	Matrix3d OMIGA_ni_n = SkewMat(w_ni_n);
	Matrix3d OMIGA_bi_b = SkewMat(w_bi_b);
	/*λ�ø���*/
	gnss_pos = gnss_pos - mat_n2l * C_b2n * lever_arm;
	/*�ٶȸ���*/
	gnss_vel = gnss_vel + C_b2n * OMIGA_bi_b * lever_arm + OMIGA_ni_n * C_b2n * lever_arm;
}

// �ο����������ĵ��İ뾶Ϊ
double ReS(double fai)
{
	double RN = cal_RN(fai);
	double ReS = RN * sqrt(cos(fai) * cos(fai) + (1 - E2_WGS84) * (1 - E2_WGS84) * sin(fai) * sin(fai));
	return ReS;
}