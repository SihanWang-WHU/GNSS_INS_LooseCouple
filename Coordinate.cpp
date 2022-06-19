/*************************************************************************
���� ��˼��
ѧ�� 2019302141082
�޸�ʱ�� 2021��11��21��
**************************************************************************/

#include <stdio.h>
#include <math.h>
#include <Windows.h>
#include <cmath>
#include "Coordinate.h"

/*************************************************************************
XYZInit XYZ�����ʼ��
���룺xyz����ṹ��ָ�룬x y z �ֱ��Ӧ����ֵ
�������
�Ѳ���
**************************************************************************/
void XYZInit(XYZ* xyz, double x, double y, double z)
{

	(*xyz).x = x;
	(*xyz).y = y;
	(*xyz).z = z;

}

/*************************************************************************
PrintXYZ ��ӡXYZ����
���룺xyz����ṹ��ָ��
�������
�Ѳ���
**************************************************************************/
void PrintXYZ(XYZ* xyz)
{
	printf("WGS84����\n");
	printf("X:\t%16.8f\n", (*xyz).x);
	printf("Y:\t%16.8f\n", (*xyz).y);
	printf("Z:\t%16.8f\n", (*xyz).z);
}

/*************************************************************************
BLH2XYZ BLHת��ΪXYZ����
���룺xyz����ṹ��ָ�룬blh����ṹ��ָ��
�����0Ϊ����ת����1Ϊ����δ��ʼ��
�Ѳ���
**************************************************************************/
int BLH2XYZ(XYZ* xyz, BLH* blh)
{
	double e2;
	//ѡ��WGS84 ���� CGCS2000����
	e2 = E2_WGS84;

	double N = R_WGS84 / sqrt(1 - e2 * sin((*blh).lat / 180.0 * PI) * sin((*blh).lat / 180.0 * PI));
	double x = (N + (*blh).h) * cos((*blh).lat / 180.0 * PI) * cos((*blh).lng / 180.0 * PI);
	double y = (N + (*blh).h) * cos((*blh).lat / 180.0 * PI) * sin((*blh).lng / 180.0 * PI);
	double z = (N * (1 - e2) + (*blh).h) * sin((*blh).lat / 180.0 * PI);
	XYZInit(xyz, x, y, z);

	return 0;
}

/*************************************************************************
BLHInit BLH�����ʼ��
���룺blh����ṹ��ָ�룬lat lng h��Ӧγ�� ���� �߳�
�������
�Ѳ���
**************************************************************************/
void BLHInit(BLH* blh, double lat, double lng, double h)
{
	(*blh).lat = lat;
	(*blh).lng = lng;
	(*blh).h = h;
}

/*************************************************************************
PrintBLH ��ӡBLH����
���룺blh����ṹ��ָ��
�������
�Ѳ���
**************************************************************************/
void PrintBLH(BLH* blh)
{
	printf("WGS84����\n");
	printf("B:\t%16.12f\n", (*blh).lat);
	printf("L:\t%16.12f\n", (*blh).lng);
	printf("H:\t%16.8f\n", (*blh).h);
}

/*************************************************************************
XYZ2BLH XYZ����ת��BLH����
���룺blh����ṹ��ָ�룬xyz����ṹ��
�����0����ת����1Ϊ����δ��ʼ����xΪ0���޷����г�������
�Ѳ���
**************************************************************************/
int XYZ2BLH(BLH* blh, XYZ* xyz)
{

	double a = R_WGS84;
int j;
double delta_Z[2], N, e2, B, L, H, X, Y, Z;
X = xyz->x;
Y = xyz->y;
Z = xyz->z;

if ((X * X + Y * Y + Z * Z) < 1e-8)
{
	blh->lat = 0.0;
	blh->lng = 0.0;
	blh->h = -a;
	return 0;
}
e2 = E2_WGS84;
delta_Z[0] = e2 * Z;

blh->lng = atan2(Y, X);
for (j = 0; j < 10; j++)
{
	blh->lat = atan2(Z + delta_Z[0], sqrt(X * X + Y * Y));
	N = a / (sqrt(1 - e2 * sin(blh->lat) * sin(blh->lat)));
	blh->h = sqrt(X * X + Y * Y + (Z + delta_Z[0]) * (Z + delta_Z[0])) - N;
	delta_Z[1] = N * e2 * (Z + delta_Z[0]) / (sqrt(X * X + Y * Y + (Z + delta_Z[0]) * (Z + delta_Z[0])));

	if (abs(delta_Z[1] - delta_Z[0]) < 1e-10) break;
	else delta_Z[0] = delta_Z[1];
}

if (abs(delta_Z[1] - delta_Z[0]) >= 1e-10)
{
	return 0;
}
else
{
	blh->lat = atan2(Z + delta_Z[1], sqrt(X * X + Y * Y));
	N = a / (sqrt(1 - e2 * sin(blh->lat) * sin(blh->lat)));
	blh->h = sqrt(X * X + Y * Y + (Z + delta_Z[1]) * (Z + delta_Z[1])) - N;

	blh->lat = blh->lat * 180 / PI;
	blh->lng = blh->lng * 180 / PI;
	return 1;
}	
}




/*************************************************************************
XYZ2NEU XYZ����ת��NEU����
���룺xyzCenter��վ����ṹ��ָ�룬xyzSat��������ṹ��ָ�룬neuΪNEU�ṹ��ָ��
�����0����ת����1��ʾ��a)�����վ�������������겻��ͬһ��������; b)XYZתBLHʧ��
�Ѳ���
**************************************************************************/
int XYZ2NEU(XYZ* xyzCenter, XYZ* xyzSat, NEU* neu)
{
	double dx, dy, dz, b, l;
	int rc = 0;
	BLH blhCenter;
	dx = xyzSat->x - xyzCenter->x;
	dy = xyzSat->y - xyzCenter->y;
	dz = xyzSat->z - xyzCenter->z;
	rc = XYZ2BLH(&blhCenter, xyzCenter);
	//if (rc)
	//	return 1;
	b = blhCenter.lat / 180.0 * PI;
	l = blhCenter.lng / 180.0 * PI;
	neu->N = -sin(b) * cos(l) * dx - sin(b) * sin(l) * dy + cos(b) * dz;
	neu->E = -sin(l) * dx + cos(l) * dy;
	neu->U = cos(b) * cos(l) * dx + cos(b) * sin(l) * dy + sin(b) * dz;

	return 0;
}


/*************************************************************************
NEU2SAE NEU����ת��SAE����
���룺neuΪNEU����ṹ��ָ�룬saeΪSAE����ṹ��ָ��
�����0����ת����1��ʾSΪ0��NΪ0
�Ѳ���
**************************************************************************/
int NEU2SAE(NEU* neu, SAE* sae)
{
	double S;
	S = sqrt(pow(neu->N, 2) + pow(neu->E, 2) + pow(neu->U, 2));
	if (S == 0 || neu->N == 0)
		return 1;

	sae->S = S;
	sae->A = atan2(neu->E, neu->N);
	sae->E = asin(neu->U / S);

	return 0;
}
