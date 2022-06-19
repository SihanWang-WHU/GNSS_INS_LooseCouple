#pragma once
#ifndef _DECODE_IMR_
#define _DECODE_IMR_
#include <fstream>
#include <iostream>
#include <iomanip>
#include "ConstNums.h"
using namespace std;

/*------ imr header information ------*/
struct IMRHeader {
	bool        imr_header_status = false; //to judge whether the header is read or not. true if being read.
	char        szHeader[8];//NULL terminated ASCII string
	int8_t      bIsIntelOrMotorola;//0:Little Endian,1:Big Endian
	double      dVersionNumber;//Inertial Explorer program version number (e.g. 8.80)
	int32_t     bDeltaTheta;
	int32_t     bDeltaVelocity;
	double      dDataRateHz;
	double      dGyroScaleFactor;
	double      dAccelScaleFactor;
	int32_t     iUtcOrGpsTime;
	int32_t     iRcvTimeOrCorrTime;
	double      dTimeTagBias;
	char        szImuName[32];
	uint8_t     blank[4];
	char        szProgramName[32];
	char        tCreate[12];//should be time_type for 12 bytes
	bool        bLeverArmValid;//true if lever arms from IMU to primary GNSS antenna are stored in this header
	int32_t     lXoffset;
	int32_t     lYoffset;
	int32_t     lZoffset;
	int8_t      reserved[354];
};


/*------ imr direct information ------*/
// 这个结构体中存放的是直接从IMR中读取进来的数据，即没有进行尺度的变化
// 从IMR中直接读进来的数据的坐标系是NEU坐标系,是原始陀螺的ZXY观测值
struct IMRDirectdata {
	double time; // GPS周内秒
	int32_t gx;
	int32_t gy;
	int32_t gz; // 尺度变化前的读数
	int32_t ax;
	int32_t ay;
	int32_t az; // 尺度变化前的读数
};

/*------ raw IMU data ------*/
struct IMURawData
{
	double  time;// GPS周内秒
	double  gx;
	double  gy;
	double  gz; // 尺度变化后的读数 (deg/s)
	double  ax;
	double  ay;
	double  az; // 尺度变化后的读数 (m/s2)
};

class IMRDATA
{
public:
/*------ Data Struct ------*/
	IMRHeader header;
	IMRDirectdata rawdata;
	IMURawData imudata;

/*------ Functions ------*/
	// 类的构造函数
	IMRDATA(FILE* imr_fp)
	{
		if (header.imr_header_status == false)
		{
			decodeHeader(imr_fp);
		}
	};

	int decodeHeader(FILE* imr_fp);      // 解码IMR头函数
	int decodedirectdata(FILE* imr_fp);  // 解码IMR中不带尺度的数据信息
	void Processdirectdata();            // 对IMR数据执行乘上尺度因子等操作

private:
};

#endif