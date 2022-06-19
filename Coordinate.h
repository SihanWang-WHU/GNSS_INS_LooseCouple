/*************************************************************************
���ƣ�����ת��ģ��ͷ�ļ�
���� ��˼��
ѧ�� 2019302141082
�޸�ʱ�� 2021��11��20��
**************************************************************************/
#pragma once
#ifndef COORDINATE_H
#define COORDINATE_H

#include "ConstNums.h"

//BLH�ṹ��  ��λΪ�Ƕ�
struct BLH
{
	double lat;
	double lng;
	double h;
};

//XYZ�ṹ��
struct XYZ
{
	double x;
	double y;
	double z;
};

//NEU�ṹ��
struct NEU
{
	double N;
	double E;
	double U;
};

//վ�ļ�����SAE
struct SAE
{
	double S;   //����
	double A;   //��λ��
	double E;   //�߶Ƚ�
};

//XYZ��ʼ��
void XYZInit(XYZ* xyz, double x, double y, double z, int flag);
//��ӡXYZ����
void PrintXYZ(XYZ* xyz);
//BLHת��ΪXYZ
int BLH2XYZ(XYZ* xyz, BLH* blh);

//BLH��ʼ��
void BLHInit(BLH* blh, double lat, double lng, double h, int flag);
//��ӡBLH����
void PrintBLH(BLH* blh);
//XY��ת��BLH
int XYZ2BLH(BLH* blh, XYZ* xyz);

//XYZ ת��ΪNEU
int XYZ2NEU(XYZ* xyzCenter, XYZ* xyzSat, NEU* neu);
//NEUת��Ϊվ�ļ�����ϵ
int NEU2SAE(NEU* neu, SAE* sae);
#endif