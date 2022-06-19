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

// 每一个历元的GNSS数据
struct GNSSEpoch 
{
	int epoch_index;//卫星结果历元序号
	double tow;//GPS周秒
	double sampling_interval;//与上一历元的GNSS观测时的时间间隔（首个历元设1）
	Vector3d gnss_pos;//纬度，经度，(rad)高程 LLH坐标系
	Vector3d gnss_vel;//北东地速度
    Matrix3d Dxx;//经纬高结果方差阵
	Matrix3d Dvv;//北东地结果方差阵
};

// GNSS所有的数据
struct GNSSALLDATA
{
	Matrix<double, Dynamic, 2> GPST;
	Matrix<double, Dynamic, 3> Pos; // 纬经高（rad rad m）
	Matrix<double, Dynamic, 3> Vel; // ENU
	Matrix<double, Dynamic, 3> Pos_std; //ENU
	Matrix<double, Dynamic, 3> V_std; //ENU
	int epoch;
};

struct GNSSRAWDATA
{
	int    rows     = 0;        //GNSS数据长度
	bool   readhead = false;
	GNSSALLDATA Gnssalldata;
	GNSSEpoch Gnssresult;
};

// 读取posT文件中的所有数据
// posT数据的排列方式是：week， sow， 纬经（deg）高，ENU坐标系下的速度，ENU坐标系下位置的中误差，ENU坐标系下速度的中误差
// 注意： Gnss_all_data是从posT文件中解码出来的数据
// 坐标为东北天
void readGNSSFile(GNSSRAWDATA* gnssrawdata, std::string name);

// 读取当前历元的数据
void getGNSSres(int epoch_index, GNSSRAWDATA* gnssrawdata);
#endif 
