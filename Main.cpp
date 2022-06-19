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
	/********************************** 数据读取和初始化 **********************************/
	// 获取IMU的数据
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
	// 获取GNSS的所有数据
	GNSSRAWDATA* gnssrawdata = new GNSSRAWDATA;
	string GNSSfp = "GNSSWQF.posT";
	// string GNSSfp = "GNSS_PLAYGROUND1.posT";
	readGNSSFile(gnssrawdata, GNSSfp);
	int gnss_epoch_index = 0;
	// 输出文件的路径
	//string outputfp = "output_playground1.txt";
	string outputfp = "output_WQF.txt";
	ofstream outresfile(outputfp);
	// 记录基站的位置
	XYZ Basexyz;
	BLH Baseblh;
	Baseblh.lat = BaseB;
	Baseblh.lng = BaseL;
	Baseblh.h = BaseH;
	BLH2XYZ(&Basexyz, &Baseblh);

	// 初始化第一个历元的位置
	double B_init = Deg2Rad(gnssrawdata->Gnssalldata.Pos(0, 0));
	double L_init = Deg2Rad(gnssrawdata->Gnssalldata.Pos(0, 1));
	double H_init = gnssrawdata->Gnssalldata.Pos(0, 2);
	double g_init = Cal_Gravity(B_init, H_init);

	// 构造IMR文件数据的对象
	IMRDATA imr_data(imr_fp);

	// 松组合准备
	Vector3d g_bias; g_bias << 0.0, 0.0, 0.0;//陀螺零偏（先置0）
	Vector3d a_bias; a_bias << 0.0, 0.0, 0.0;//加计零偏（先置0）
	Vector3d g_s; g_s << 0.0, 0.0, 0.0;//陀螺比例因子（先置1）
	Vector3d a_s; a_s << 0.0, 0.0, 0.0;//加计比例因子（先置1）
	Vector3d imu_pos;
	Vector3d imu_vel;
	Vector3d imu_eul;
	Vector3d f_bi_b;
	Vector3d w_bi_b;

	Vector3d lever_arm;
	lever_arm << lever_arm_forward, lever_arm_right, lever_arm_down;//前右下杆臂值

	LCKF kalmanfilter;
	kalmanfilter.InitKalman();

	/********************************** 初始对准 **********************************/

	//将INIT_ALIGN_EPOCH个历元的IMR数据装载用于初始对准
	InitAlign init_align;
	MatrixXd raw_accel_for_init(InitEpochs, 3);
	MatrixXd raw_gyro_for_init(InitEpochs, 3);

	// 读取数据，初始化Init矩阵
	for (int i = 0; i < InitEpochs; i++)
	{
		imr_data.decodedirectdata(imr_fp);
		imr_data.Processdirectdata();
		// 这里需要注意IMR中文件的坐标系是右前上坐标系
		// 利用下列的操作将坐标系转换成了NED坐标系
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
	Vector3d Euler = RotationMatrixToEuler(rot_mat);//得到初始姿态角 roll（横滚） pitch（俯仰） yaw（航向）
	cout << " Roll = " << Rad2Deg(Euler(0)) << " Pitch = " << Rad2Deg(Euler(1))
		 << " Heading = " << Rad2Deg(Euler(2)) << endl;
	Quaterniond q_init = EulerToQuaternion(Euler);// 四元数

	/********************************** 机械编排 **********************************/
	// 初始化第一个历元的姿态、速度、位置

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


	// 初始化这个历元的结果
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
	
	// 第一个历元视作现在的result就是原来的
	ins_res_bef = ins_res_now;

	// 将data存储进来
	// 要注意的是：
	// IMR_DATA.IMUDATA   这里面存放的是原始的IMR数据直接解码得到的IMU数据，坐标系为ENU，且不是增量形式
	// IMU_NED_DATA       这里面存放的是坐标系为NED的数据，要进行坐标的转换，且要变换成增量形式
	imudata_now.time = imr_data.imudata.time;
	imudata_now.accel_x = imr_data.imudata.ay * IMU_SAMPLE_INTERVAL;
	imudata_now.accel_y = imr_data.imudata.ax * IMU_SAMPLE_INTERVAL;
	imudata_now.accel_z = -imr_data.imudata.az * IMU_SAMPLE_INTERVAL;
	imudata_now.gyro_x = Deg2Rad(imr_data.imudata.gy * IMU_SAMPLE_INTERVAL);
	imudata_now.gyro_y = Deg2Rad(imr_data.imudata.gx * IMU_SAMPLE_INTERVAL);
	imudata_now.gyro_z = Deg2Rad(-imr_data.imudata.gz * IMU_SAMPLE_INTERVAL);

	/*寻找当前惯导结果后的下一个GNSS结果位置*/
	for (int i = 0; i < gnssrawdata->Gnssalldata.epoch; i++)
	{
		double gnss_t = gnssrawdata->Gnssalldata.GPST(i, 1);
		if ((imudata_now.time - gnss_t) < 0 && (imudata_now.time - gnss_t) > -1.0)
		{
			gnss_epoch_index = i;//接下来的GNSS的历元序号
			break;
		}
	}

	// 程序总体的跨度是以IMU为主的
	while (imr_fp)
	{
		// before = now
		ins_res_saf = ins_res_bef;
		ins_res_bef = ins_res_now;
		imudata_saf = imudata_bef;
		imudata_bef = imudata_now;

		// IMR原始数据的读取
		imr_data.decodedirectdata(imr_fp);
		imr_data.Processdirectdata();

		// 转换成可使用的增量形式
		imudata_now.time = imr_data.imudata.time;
		imudata_now.accel_x = imr_data.imudata.ay * IMU_SAMPLE_INTERVAL;
		imudata_now.accel_y = imr_data.imudata.ax * IMU_SAMPLE_INTERVAL;
		imudata_now.accel_z = -imr_data.imudata.az * IMU_SAMPLE_INTERVAL;
		imudata_now.gyro_x = Deg2Rad(imr_data.imudata.gy * IMU_SAMPLE_INTERVAL);
		imudata_now.gyro_y = Deg2Rad(imr_data.imudata.gx * IMU_SAMPLE_INTERVAL);
		imudata_now.gyro_z = Deg2Rad(-imr_data.imudata.gz * IMU_SAMPLE_INTERVAL);

		//反馈调节零偏
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

		// 惯导机械编排
		// 更新的顺序依次为姿态->速度->位置
		Attitude_Updating(&imudata_bef, &imudata_now, &ins_res_bef, &ins_res_now);
		Velocity_Updating(&imudata_saf, &imudata_bef, &imudata_now, &ins_res_saf, &ins_res_bef, &ins_res_now);
		Position_Updating(&imudata_bef, &imudata_now, &ins_res_bef, &ins_res_now);

		/********************************** 一步预测 **********************************/
		imu_pos << ins_res_bef.position.B, ins_res_bef.position.L, ins_res_bef.position.H;
		imu_vel << ins_res_bef.velocity.VN, ins_res_bef.velocity.VE, ins_res_bef.velocity.VD;
		imu_eul << ins_res_bef.attitude.E.Roll, ins_res_bef.attitude.E.Pitch, ins_res_bef.attitude.E.Yaw;
		// 加速度计的比力输出
		f_bi_b << imudata_bef.accel_x * IMU_SAMPLE_HZ,
				  imudata_bef.accel_y * IMU_SAMPLE_HZ,
				  imudata_bef.accel_z * IMU_SAMPLE_HZ;
		// 陀螺仪的非增量输出
		w_bi_b << imudata_bef.gyro_x * IMU_SAMPLE_HZ,
				  imudata_bef.gyro_y * IMU_SAMPLE_HZ,
				  imudata_bef.gyro_z * IMU_SAMPLE_HZ;
		double dT = imudata_now.time - imudata_bef.time;

		kalmanfilter.processPredict(imu_pos, imu_vel, imu_eul, f_bi_b, w_bi_b, dT);

		/********************************** 量测更新 **********************************/
		if (fabs(imudata_now.time - gnssrawdata->Gnssalldata.GPST(gnss_epoch_index, 1)) < 0.005 
			&& gnss_epoch_index <= gnssrawdata->rows)
		{
			getGNSSres(gnss_epoch_index, gnssrawdata);
			gnss_epoch_index++;
			imu_pos << ins_res_now.position.B, ins_res_now.position.L, ins_res_now.position.H;
			imu_vel << ins_res_now.velocity.VN, ins_res_now.velocity.VE, ins_res_now.velocity.VD;
			imu_eul << ins_res_now.attitude.E.Roll, ins_res_now.attitude.E.Pitch, ins_res_now.attitude.E.Yaw;
			// 加速度计的比力输出
			f_bi_b << imudata_now.accel_x * IMU_SAMPLE_HZ,
					  imudata_now.accel_y * IMU_SAMPLE_HZ,
					  imudata_now.accel_z * IMU_SAMPLE_HZ;
			// 陀螺仪的非增量输出
			w_bi_b << imudata_now.gyro_x * IMU_SAMPLE_HZ,
					  imudata_now.gyro_y * IMU_SAMPLE_HZ,
					  imudata_now.gyro_z * IMU_SAMPLE_HZ;
			////杆臂改正
			//correctLeverArm(gnssrawdata->Gnssresult.gnss_pos, gnssrawdata->Gnssresult.gnss_vel, 
			//	            imu_pos, imu_vel, imu_eul, w_bi_b, lever_arm);
			/*再用机械编排新值对其进行更新*/
			kalmanfilter.processUpdate(imu_pos, imu_vel, imu_eul, g_bias, a_bias, g_s, a_s, gnssrawdata->Gnssresult.gnss_pos,
				                       gnssrawdata->Gnssresult.gnss_vel, gnssrawdata->Gnssresult.Dxx, gnssrawdata->Gnssresult.Dvv, lever_arm, w_bi_b);
			//结果反赋给机械编排的结果结构体
			ins_res_now.position.B = imu_pos(0), ins_res_now.position.L = imu_pos(1), ins_res_now.position.H = imu_pos(2);
			ins_res_now.velocity.VN = imu_vel(0), ins_res_now.velocity.VE = imu_vel(1), ins_res_now.velocity.VD = imu_vel(2);
			ins_res_now.attitude.E.Roll = imu_eul(0), ins_res_now.attitude.E.Pitch = imu_eul(1), ins_res_now.attitude.E.Yaw = imu_eul(2);
			Quaterniond q;
			q = EulerToQuaternion(imu_eul);
			ins_res_now.attitude.Q = q;

			// 将状态置零
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