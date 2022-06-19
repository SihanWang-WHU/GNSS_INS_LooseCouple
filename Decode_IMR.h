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
// ����ṹ���д�ŵ���ֱ�Ӵ�IMR�ж�ȡ���������ݣ���û�н��г߶ȵı仯
// ��IMR��ֱ�Ӷ����������ݵ�����ϵ��NEU����ϵ,��ԭʼ���ݵ�ZXY�۲�ֵ
struct IMRDirectdata {
	double time; // GPS������
	int32_t gx;
	int32_t gy;
	int32_t gz; // �߶ȱ仯ǰ�Ķ���
	int32_t ax;
	int32_t ay;
	int32_t az; // �߶ȱ仯ǰ�Ķ���
};

/*------ raw IMU data ------*/
struct IMURawData
{
	double  time;// GPS������
	double  gx;
	double  gy;
	double  gz; // �߶ȱ仯��Ķ��� (deg/s)
	double  ax;
	double  ay;
	double  az; // �߶ȱ仯��Ķ��� (m/s2)
};

class IMRDATA
{
public:
/*------ Data Struct ------*/
	IMRHeader header;
	IMRDirectdata rawdata;
	IMURawData imudata;

/*------ Functions ------*/
	// ��Ĺ��캯��
	IMRDATA(FILE* imr_fp)
	{
		if (header.imr_header_status == false)
		{
			decodeHeader(imr_fp);
		}
	};

	int decodeHeader(FILE* imr_fp);      // ����IMRͷ����
	int decodedirectdata(FILE* imr_fp);  // ����IMR�в����߶ȵ�������Ϣ
	void Processdirectdata();            // ��IMR����ִ�г��ϳ߶����ӵȲ���

private:
};

#endif