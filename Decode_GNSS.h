#pragma once
#ifndef _DECODE_GNSS_H_
#define _DECODE_GNSS_H_
#include<fstream>
#include<iostream>
#include<sstream> 
#include <Eigen/Dense>
#include <Eigen/Core>
#include"ConstNums.h"
#include"Data_Functions.h"

using namespace Eigen;

// ÿһ����Ԫ��GNSS����
struct GNSSEpoch 
{
	int epoch_index;//���ǽ����Ԫ���
	double tow;//GPS����
	double sampling_interval;//����һ��Ԫ��GNSS�۲�ʱ��ʱ�������׸���Ԫ��1��
	Vector3d gnss_pos;//γ�ȣ����ȣ�(rad)�߳� LLH����ϵ
	Vector3d gnss_vel;//�������ٶ�
    Matrix3d Dxx;//��γ�߽��������
	Matrix3d Dvv;//�����ؽ��������
};

// GNSS���е�����
struct GNSSALLDATA
{
	Matrix<double, Dynamic, 2> GPST;
	Matrix<double, Dynamic, 3> Pos; // γ���ߣ�rad rad m��
	Matrix<double, Dynamic, 3> Vel; // ENU
	Matrix<double, Dynamic, 3> Pos_std; //ENU
	Matrix<double, Dynamic, 3> V_std; //ENU
	int epoch;
};

struct GNSSRAWDATA
{
	int    rows     = 0;        //GNSS���ݳ���
	bool   readhead = false;
	GNSSALLDATA Gnssalldata;
	GNSSEpoch Gnssresult;
};

// ��ȡposT�ļ��е���������
// posT���ݵ����з�ʽ�ǣ�week�� sow�� γ����deg���ߣ�ENU����ϵ�µ��ٶȣ�ENU����ϵ��λ�õ�����ENU����ϵ���ٶȵ������
// ע�⣺ Gnss_all_data�Ǵ�posT�ļ��н������������
// ����Ϊ������
void readGNSSFile(GNSSRAWDATA* gnssrawdata, std::string name);

// ��ȡ��ǰ��Ԫ������
void getGNSSres(int epoch_index, GNSSRAWDATA* gnssrawdata);
#endif 
