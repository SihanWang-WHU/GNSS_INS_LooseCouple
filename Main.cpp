#define  _CRT_SECURE_NO_WARNINGS

#include <iostream>
#include<iomanip>
#include<fstream>
#include"Initial_Allignment.h"
#include"Data_Functions.h"
#include"ins_mech.h"
#include"Coordinate.h"
#include"Config.h"
#include"Decode_GNSS.h"
#include"Loose_Couple.h"


using namespace std;

int main()
{
	/********************************** ���ݶ�ȡ�ͳ�ʼ�� **********************************/
	// ��ȡIMU������
	FILE* imr_fp;
	imr_fp = fopen("WQF.imr", "rb");
	// imr_fp = fopen("static.imr", "rb");      /*Open imu file for reading*/
	/*imr_fp = fopen("kinematic2_sbf_imu.imr", "rb");*/
	// imr_fp = fopen("IMU_PLAYGROUND1.imr", "rb");
	if (imr_fp == NULL)
	{
		cout << "Cannot Open the IMR File." << endl;
		return -1;
	}
	// ��ȡGNSS����������
	GNSSRAWDATA* gnssrawdata = new GNSSRAWDATA;
	string GNSSfp = "GNSSWQF.posT";
	// string GNSSfp = "GNSS_PLAYGROUND1.posT";
	readGNSSFile(gnssrawdata, GNSSfp);
	int gnss_epoch_index = 0;
	// ����ļ���·��
	//string outputfp = "output_playground1.txt";
	string outputfp = "output_WQF.txt";
	ofstream outresfile(outputfp);
	// ��¼��վ��λ��
	XYZ Basexyz;
	BLH Baseblh;
	Baseblh.lat = BaseB;
	Baseblh.lng = BaseL;
	Baseblh.h = BaseH;
	BLH2XYZ(&Basexyz, &Baseblh);

	// ��ʼ����һ����Ԫ��λ��
	double B_init = Deg2Rad(gnssrawdata->Gnssalldata.Pos(0, 0));
	double L_init = Deg2Rad(gnssrawdata->Gnssalldata.Pos(0, 1));
	double H_init = gnssrawdata->Gnssalldata.Pos(0, 2);
	double g_init = Cal_Gravity(B_init, H_init);

	// ����IMR�ļ����ݵĶ���
	IMRDATA imr_data(imr_fp);

	// �����׼��
	Vector3d g_bias; g_bias << 0.0, 0.0, 0.0;//������ƫ������0��
	Vector3d a_bias; a_bias << 0.0, 0.0, 0.0;//�Ӽ���ƫ������0��
	Vector3d g_s; g_s << 0.0, 0.0, 0.0;//���ݱ������ӣ�����1��
	Vector3d a_s; a_s << 0.0, 0.0, 0.0;//�ӼƱ������ӣ�����1��
	Vector3d imu_pos;
	Vector3d imu_vel;
	Vector3d imu_eul;
	Vector3d f_bi_b;
	Vector3d w_bi_b;

	Vector3d lever_arm;
	lever_arm << lever_arm_forward, lever_arm_right, lever_arm_down;//ǰ���¸˱�ֵ

	LCKF kalmanfilter;
	kalmanfilter.InitKalman();

	/********************************** ��ʼ��׼ **********************************/

	//��INIT_ALIGN_EPOCH����Ԫ��IMR����װ�����ڳ�ʼ��׼
	InitAlign init_align;
	MatrixXd raw_accel_for_init(InitEpochs, 3);
	MatrixXd raw_gyro_for_init(InitEpochs, 3);

	// ��ȡ���ݣ���ʼ��Init����
	for (int i = 0; i < InitEpochs; i++)
	{
		imr_data.decodedirectdata(imr_fp);
		imr_data.Processdirectdata();
		// ������Ҫע��IMR���ļ�������ϵ����ǰ������ϵ
		// �������еĲ���������ϵת������NED����ϵ
		// X = - z
		// Y = x
		// Z = - y
		raw_accel_for_init(i, 0) = imr_data.imudata.ay;
		raw_accel_for_init(i, 1) = imr_data.imudata.ax;
		raw_accel_for_init(i, 2) = -imr_data.imudata.az;
		raw_gyro_for_init(i, 0) = Deg2Rad(imr_data.imudata.gy);
		raw_gyro_for_init(i, 1) = Deg2Rad(imr_data.imudata.gx);
		raw_gyro_for_init(i, 2) = Deg2Rad(-imr_data.imudata.gz);
	}

	init_align.set(raw_accel_for_init, raw_gyro_for_init, g_init, B_init);

	Matrix3d rot_mat = init_align.initAlign();
	Vector3d Euler = RotationMatrixToEuler(rot_mat);//�õ���ʼ��̬�� roll������� pitch�������� yaw������
	cout << " Roll = " << Rad2Deg(Euler(0)) << " Pitch = " << Rad2Deg(Euler(1))
		 << " Heading = " << Rad2Deg(Euler(2)) << endl;
	Quaterniond q_init = EulerToQuaternion(Euler);// ��Ԫ��

	/********************************** ��е���� **********************************/
	// ��ʼ����һ����Ԫ����̬���ٶȡ�λ��

	INS_Result ins_res_now;
	INS_Result ins_res_bef;
	INS_Result ins_res_saf;

	IMU_NED_DATA imudata_now;
	IMU_NED_DATA imudata_bef;
	IMU_NED_DATA imudata_saf;

	memset(&ins_res_saf, 0.0, sizeof(INS_Result));
	memset(&ins_res_bef, 0.0, sizeof(INS_Result));
	memset(&ins_res_now, 0.0, sizeof(INS_Result));
	memset(&imudata_saf, 0.0, sizeof(IMU_NED_DATA));
	memset(&imudata_bef, 0.0, sizeof(IMU_NED_DATA));
	memset(&imudata_now, 0.0, sizeof(IMU_NED_DATA));


	// ��ʼ�������Ԫ�Ľ��
	ins_res_now.attitude.Q.w() = q_init.w();
	ins_res_now.attitude.Q.x() = q_init.x();
	ins_res_now.attitude.Q.y() = q_init.y();
	ins_res_now.attitude.Q.z() = q_init.z();
	ins_res_now.attitude.E.Roll = Euler(0);
	ins_res_now.attitude.E.Pitch = Euler(1);
	ins_res_now.attitude.E.Yaw = Euler(2);
	ins_res_now.velocity.VN = 0;
	ins_res_now.velocity.VE = 0;
	ins_res_now.velocity.VD = 0;
	ins_res_now.position.B = B_init;
	ins_res_now.position.L = L_init;
	ins_res_now.position.H = H_init;
	
	// ��һ����Ԫ�������ڵ�result����ԭ����
	ins_res_bef = ins_res_now;

	// ��data�洢����
	// Ҫע����ǣ�
	// IMR_DATA.IMUDATA   �������ŵ���ԭʼ��IMR����ֱ�ӽ���õ���IMU���ݣ�����ϵΪENU���Ҳ���������ʽ
	// IMU_NED_DATA       �������ŵ�������ϵΪNED�����ݣ�Ҫ���������ת������Ҫ�任��������ʽ
	imudata_now.time = imr_data.imudata.time;
	imudata_now.accel_x = imr_data.imudata.ay * IMU_SAMPLE_INTERVAL;
	imudata_now.accel_y = imr_data.imudata.ax * IMU_SAMPLE_INTERVAL;
	imudata_now.accel_z = -imr_data.imudata.az * IMU_SAMPLE_INTERVAL;
	imudata_now.gyro_x = Deg2Rad(imr_data.imudata.gy * IMU_SAMPLE_INTERVAL);
	imudata_now.gyro_y = Deg2Rad(imr_data.imudata.gx * IMU_SAMPLE_INTERVAL);
	imudata_now.gyro_z = Deg2Rad(-imr_data.imudata.gz * IMU_SAMPLE_INTERVAL);

	/*Ѱ�ҵ�ǰ�ߵ���������һ��GNSS���λ��*/
	for (int i = 0; i < gnssrawdata->Gnssalldata.epoch; i++)
	{
		double gnss_t = gnssrawdata->Gnssalldata.GPST(i, 1);
		if ((imudata_now.time - gnss_t) < 0 && (imudata_now.time - gnss_t) > -1.0)
		{
			gnss_epoch_index = i;//��������GNSS����Ԫ���
			break;
		}
	}

	// ��������Ŀ������IMUΪ����
	while (imr_fp)
	{
		// before = now
		ins_res_saf = ins_res_bef;
		ins_res_bef = ins_res_now;
		imudata_saf = imudata_bef;
		imudata_bef = imudata_now;

		// IMRԭʼ���ݵĶ�ȡ
		imr_data.decodedirectdata(imr_fp);
		imr_data.Processdirectdata();

		// ת���ɿ�ʹ�õ�������ʽ
		imudata_now.time = imr_data.imudata.time;
		imudata_now.accel_x = imr_data.imudata.ay * IMU_SAMPLE_INTERVAL;
		imudata_now.accel_y = imr_data.imudata.ax * IMU_SAMPLE_INTERVAL;
		imudata_now.accel_z = -imr_data.imudata.az * IMU_SAMPLE_INTERVAL;
		imudata_now.gyro_x = Deg2Rad(imr_data.imudata.gy * IMU_SAMPLE_INTERVAL);
		imudata_now.gyro_y = Deg2Rad(imr_data.imudata.gx * IMU_SAMPLE_INTERVAL);
		imudata_now.gyro_z = Deg2Rad(-imr_data.imudata.gz * IMU_SAMPLE_INTERVAL);

		//����������ƫ
		imudata_now.accel_x -= IMU_SAMPLE_INTERVAL * a_bias(0);
		imudata_now.accel_y -= IMU_SAMPLE_INTERVAL * a_bias(1);
		imudata_now.accel_z -= IMU_SAMPLE_INTERVAL * a_bias(2);
		imudata_now.gyro_x  -= IMU_SAMPLE_INTERVAL * g_bias(0);
		imudata_now.gyro_y  -= IMU_SAMPLE_INTERVAL * g_bias(1);
		imudata_now.gyro_z  -= IMU_SAMPLE_INTERVAL * g_bias(2);
		imudata_now.accel_x = imudata_now.accel_x / (a_s(0) + 1);
		imudata_now.accel_y = imudata_now.accel_y / (a_s(1) + 1);
		imudata_now.accel_z = imudata_now.accel_z / (a_s(2) + 1);
		imudata_now.gyro_x  = imudata_now.gyro_x / (g_s(0) + 1);
		imudata_now.gyro_y  = imudata_now.gyro_y / (g_s(1) + 1);
		imudata_now.gyro_z  = imudata_now.gyro_z / (g_s(2) + 1);

		// �ߵ���е����
		// ���µ�˳������Ϊ��̬->�ٶ�->λ��
		Attitude_Updating(&imudata_bef, &imudata_now, &ins_res_bef, &ins_res_now);
		Velocity_Updating(&imudata_saf, &imudata_bef, &imudata_now, &ins_res_saf, &ins_res_bef, &ins_res_now);
		Position_Updating(&imudata_bef, &imudata_now, &ins_res_bef, &ins_res_now);

		/********************************** һ��Ԥ�� **********************************/
		imu_pos << ins_res_bef.position.B, ins_res_bef.position.L, ins_res_bef.position.H;
		imu_vel << ins_res_bef.velocity.VN, ins_res_bef.velocity.VE, ins_res_bef.velocity.VD;
		imu_eul << ins_res_bef.attitude.E.Roll, ins_res_bef.attitude.E.Pitch, ins_res_bef.attitude.E.Yaw;
		// ���ٶȼƵı������
		f_bi_b << imudata_bef.accel_x * IMU_SAMPLE_HZ,
				  imudata_bef.accel_y * IMU_SAMPLE_HZ,
				  imudata_bef.accel_z * IMU_SAMPLE_HZ;
		// �����ǵķ��������
		w_bi_b << imudata_bef.gyro_x * IMU_SAMPLE_HZ,
				  imudata_bef.gyro_y * IMU_SAMPLE_HZ,
				  imudata_bef.gyro_z * IMU_SAMPLE_HZ;
		double dT = imudata_now.time - imudata_bef.time;

		kalmanfilter.processPredict(imu_pos, imu_vel, imu_eul, f_bi_b, w_bi_b, dT);

		/********************************** ������� **********************************/
		if (fabs(imudata_now.time - gnssrawdata->Gnssalldata.GPST(gnss_epoch_index, 1)) < 0.005 
			&& gnss_epoch_index <= gnssrawdata->rows)
		{
			getGNSSres(gnss_epoch_index, gnssrawdata);
			gnss_epoch_index++;
			imu_pos << ins_res_now.position.B, ins_res_now.position.L, ins_res_now.position.H;
			imu_vel << ins_res_now.velocity.VN, ins_res_now.velocity.VE, ins_res_now.velocity.VD;
			imu_eul << ins_res_now.attitude.E.Roll, ins_res_now.attitude.E.Pitch, ins_res_now.attitude.E.Yaw;
			// ���ٶȼƵı������
			f_bi_b << imudata_now.accel_x * IMU_SAMPLE_HZ,
					  imudata_now.accel_y * IMU_SAMPLE_HZ,
					  imudata_now.accel_z * IMU_SAMPLE_HZ;
			// �����ǵķ��������
			w_bi_b << imudata_now.gyro_x * IMU_SAMPLE_HZ,
					  imudata_now.gyro_y * IMU_SAMPLE_HZ,
					  imudata_now.gyro_z * IMU_SAMPLE_HZ;
			////�˱۸���
			//correctLeverArm(gnssrawdata->Gnssresult.gnss_pos, gnssrawdata->Gnssresult.gnss_vel, 
			//	            imu_pos, imu_vel, imu_eul, w_bi_b, lever_arm);
			/*���û�е������ֵ������и���*/
			kalmanfilter.processUpdate(imu_pos, imu_vel, imu_eul, g_bias, a_bias, g_s, a_s, gnssrawdata->Gnssresult.gnss_pos,
				                       gnssrawdata->Gnssresult.gnss_vel, gnssrawdata->Gnssresult.Dxx, gnssrawdata->Gnssresult.Dvv, lever_arm, w_bi_b);
			//�����������е���ŵĽ���ṹ��
			ins_res_now.position.B = imu_pos(0), ins_res_now.position.L = imu_pos(1), ins_res_now.position.H = imu_pos(2);
			ins_res_now.velocity.VN = imu_vel(0), ins_res_now.velocity.VE = imu_vel(1), ins_res_now.velocity.VD = imu_vel(2);
			ins_res_now.attitude.E.Roll = imu_eul(0), ins_res_now.attitude.E.Pitch = imu_eul(1), ins_res_now.attitude.E.Yaw = imu_eul(2);
			Quaterniond q;
			q = EulerToQuaternion(imu_eul);
			ins_res_now.attitude.Q = q;

			// ��״̬����
			kalmanfilter.x.setZero();
		}

		XYZ imuxyz;
		BLH imublh;
		imublh.lat = ins_res_now.position.B * R2D;
		imublh.lng = ins_res_now.position.L * R2D;
		imublh.h = ins_res_now.position.H;
		BLH2XYZ(&imuxyz, &imublh);
		XYZ2NEU(&Basexyz, &imuxyz, &ins_res_now.position.neu);

	   cout << fixed << setprecision(10) << setw(10) << imr_data.imudata.time << "  " << ins_res_now.position.neu.N << "  " << ins_res_now.position.neu.E << "  " << ins_res_now.position.neu.U << "  "
				  << fixed << setprecision(10) << setw(10) << ins_res_now.velocity.VN << "  " << ins_res_now.velocity.VE << "  " << ins_res_now.velocity.VD << "  "
				  << fixed << setprecision(10) << setw(10) << Rad2Deg(ins_res_now.attitude.E.Roll) << "  " << Rad2Deg(ins_res_now.attitude.E.Pitch) << "  " << Rad2Deg(ins_res_now.attitude.E.Yaw) << "\n";

	   outresfile << fixed << setprecision(10) << setw(10) << imr_data.imudata.time << "  " << ins_res_now.position.neu.N << "  " << ins_res_now.position.neu.E << "  " << ins_res_now.position.neu.U << "  "
		    << fixed << setprecision(10) << setw(10) << ins_res_now.velocity.VN << "  " << ins_res_now.velocity.VE << "  " << ins_res_now.velocity.VD << "  "
		    << fixed << setprecision(10) << setw(10) << Rad2Deg(ins_res_now.attitude.E.Roll) << "  " << Rad2Deg(ins_res_now.attitude.E.Pitch) << "  " << Rad2Deg(ins_res_now.attitude.E.Yaw) << "   "
		    <<  fixed << setprecision(10) << setw(10) << ins_res_now.position.B * R2D << "  " << ins_res_now.position.L * R2D << "  " << ins_res_now.position.H << "  ""\n";
	   
	}
	outresfile.close();
	system("pause");
	return 0;
}

int printGnssData()
{
	GNSSRAWDATA* gnssrawdata = new GNSSRAWDATA;
	string GNSSfp = "kinematic2_GNSS.posT";
	readGNSSFile(gnssrawdata, GNSSfp);
	for (int i = 0; i < *(&gnssrawdata->rows); i++)
	{
 		getGNSSres(i, gnssrawdata);
		cout << " time " << gnssrawdata->Gnssresult.tow << " B " << gnssrawdata->Gnssresult.gnss_pos[0] << " L " << gnssrawdata->Gnssresult.gnss_pos[1]
			 << " H " << gnssrawdata->Gnssresult.gnss_pos[2] << " VN " << gnssrawdata->Gnssresult.gnss_vel[0] << " VE " << gnssrawdata->Gnssresult.gnss_vel[1]
			 << " VD " << gnssrawdata->Gnssresult.gnss_vel[2] << endl;
	}
	return 0;
}