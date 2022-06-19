/*************************************************************************
名称：坐标转换模块头文件
作者 王思翰
学号 2019302141082
修改时间 2021年11月20日
**************************************************************************/
#pragma once
#ifndef COORDINATE_H
#define COORDINATE_H

#include "ConstNums.h"

//BLH结构体  单位为角度
struct BLH
{
	double lat;
	double lng;
	double h;
};

//XYZ结构体
struct XYZ
{
	double x;
	double y;
	double z;
};

//NEU结构体
struct NEU
{
	double N;
	double E;
	double U;
};

//站心极坐标SAE
struct SAE
{
	double S;   //距离
	double A;   //方位角
	double E;   //高度角
};

//XYZ初始化
void XYZInit(XYZ* xyz, double x, double y, double z, int flag);
//打印XYZ坐标
void PrintXYZ(XYZ* xyz);
//BLH转化为XYZ
int BLH2XYZ(XYZ* xyz, BLH* blh);

//BLH初始化
void BLHInit(BLH* blh, double lat, double lng, double h, int flag);
//打印BLH坐标
void PrintBLH(BLH* blh);
//XY在转化BLH
int XYZ2BLH(BLH* blh, XYZ* xyz);

//XYZ 转化为NEU
int XYZ2NEU(XYZ* xyzCenter, XYZ* xyzSat, NEU* neu);
//NEU转化为站心极坐标系
int NEU2SAE(NEU* neu, SAE* sae);
#endif