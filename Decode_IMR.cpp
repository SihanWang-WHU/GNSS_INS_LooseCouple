#include"Decode_IMR.h"

using namespace std;

/*******************************************************
* 解码IMR文件中的文件头
* @paras[in]   IMR文件路径
* @paras[out]  none
* @return -1:解码失败 0:解码成功
*******************************************************/
int IMRDATA::decodeHeader(FILE* imr_fp)
{
	char buff[IMR_HEADER];
	if (fread(buff, IMR_HEADER, 1, imr_fp) < 1) {
		cout << "read imr header in file error! " << endl;
		header.imr_header_status = false;
		return -1;
	}
	memcpy(&header.szHeader, buff, 8);
	memcpy(&header.bIsIntelOrMotorola, buff + 8, 1);
	memcpy(&header.dVersionNumber, buff + 9, 8);
	memcpy(&header.bDeltaTheta, buff + 17, 4);
	memcpy(&header.bDeltaVelocity, buff + 21, 4);
	memcpy(&header.dDataRateHz, buff + 25, 8);
	memcpy(&header.dGyroScaleFactor, buff + 33, 8);
	memcpy(&header.dAccelScaleFactor, buff + 41, 8);
	memcpy(&header.iUtcOrGpsTime, buff + 49, 4);
	memcpy(&header.iRcvTimeOrCorrTime, buff + 53, 4);
	memcpy(&header.dTimeTagBias, buff + 57, 8);
	memcpy(&header.szImuName, buff + 65, 32);
	memcpy(&header.blank, buff + 97, 4);
	memcpy(&header.szProgramName, buff + 101, 32);
	memcpy(&header.tCreate, buff + 133, 12);
	memcpy(&header.bLeverArmValid, buff + 145, 1);
	memcpy(&header.lXoffset, buff + 146, 4);
	memcpy(&header.lYoffset, buff + 150, 4);
	memcpy(&header.lZoffset, buff + 154, 4);
	memcpy(&header.reserved, buff + 158, 354);
	header.imr_header_status = true;
	return 0;
}

/*******************************************************
* 解码IMR文件中直接的数据
* @paras[in]   IMR文件路径
* @paras[out]  none
* @return -1:解码失败 0:解码成功
*******************************************************/
int IMRDATA::decodedirectdata(FILE* imr_fp)
{
	if (!header.imr_header_status) {
		cout << "imr header has not been read! " << endl;
		return -1;//imr header has not been read; exit if error
	}
	char buff[IMR_RAW_DATA];
	if (fread(buff, IMR_RAW_DATA, 1, imr_fp) < 1) {
		cout << "read imr data in file error! " << endl;
		return -1;
	}
	memcpy(&rawdata.time, buff, 8);
	memcpy(&rawdata.gx, buff + 8, 4);
	memcpy(&rawdata.gy, buff + 12, 4);
	memcpy(&rawdata.gz, buff + 16, 4);
	memcpy(&rawdata.ax, buff + 20, 4);
	memcpy(&rawdata.ay, buff + 24, 4);
	memcpy(&rawdata.az, buff + 28, 4);
	return 0;
}

/*******************************************************
* 将IMR文件中直接的数据转换为IMU数据
* @paras[in]   none
* @paras[out]  none
* @return      none
*******************************************************/
void IMRDATA::Processdirectdata()
{
	if (header.iUtcOrGpsTime == 1)
	{
		imudata.time = rawdata.time + 18;//GPST-UTC=18s
	}
	else
	{
		imudata.time = rawdata.time;
	}
	/*-- imr record in deg/s and m/s2 --*/
	if (header.bDeltaTheta == 0) {
	// 把IMR中的数据乘上尺度因子
		imudata.gx = rawdata.gx * header.dGyroScaleFactor;
		imudata.gy = rawdata.gy * header.dGyroScaleFactor;
		imudata.gz = rawdata.gz * header.dGyroScaleFactor;
		imudata.ax = rawdata.ax * header.dAccelScaleFactor;
		imudata.ay = rawdata.ay * header.dAccelScaleFactor;
		imudata.az = rawdata.az * header.dAccelScaleFactor;
	}
	/*-- imr record in deg and m/s --*/
	else {
		imudata.gx = rawdata.gx * header.dGyroScaleFactor * header.dDataRateHz;
		imudata.gy = rawdata.gy * header.dGyroScaleFactor * header.dDataRateHz;
		imudata.gz = rawdata.gz * header.dGyroScaleFactor * header.dDataRateHz;
		imudata.ax = rawdata.ax * header.dAccelScaleFactor * header.dDataRateHz;
		imudata.ay = rawdata.ay * header.dAccelScaleFactor * header.dDataRateHz;
		imudata.az = rawdata.az * header.dAccelScaleFactor * header.dDataRateHz;
	}
}
