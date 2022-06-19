#include "Decode_GNSS.h"

using namespace std;


/*******************************************************
* @brief       ��ȡposT�ļ��������еĽ���浽������ȥ
* @author      Sihan Wang
* @paras[in]   GNSSRAWDATA�ṹ�壬posT�ļ�·��
* @paras[out]  none
* @return      void
* @Date        25/5/2022
*******************************************************/
void readGNSSFile(GNSSRAWDATA* gnssrawdata, string name)
{
	ifstream infile_getline;
	infile_getline.open(name, ios::in);
	ifstream infile;
	infile.open(name, ios::in);
	if (!infile_getline.is_open())
	{
		cout << "Open file failure" << endl;
	}
	string line;
	// ����ѭ���õ��ļ�������
	while (getline(infile_getline, line))
	{
		gnssrawdata->rows++;
	}
	infile_getline.close();
	// ���￼�����ļ�ͷ����-2
	gnssrawdata->rows -= 2;
	gnssrawdata->Gnssalldata.GPST.resize(gnssrawdata->rows, 2);
	gnssrawdata->Gnssalldata.Pos.resize(gnssrawdata->rows, 3);
	gnssrawdata->Gnssalldata.Vel.resize(gnssrawdata->rows, 3);
	gnssrawdata->Gnssalldata.Pos_std.resize(gnssrawdata->rows, 3);
	gnssrawdata->Gnssalldata.V_std.resize(gnssrawdata->rows, 3);


	int epoch = 0;
	while (getline(infile, line))
	{
		if (gnssrawdata->readhead == false)
		{
			for (int i = 0; i < 2; i++)
			{
				// ��ȡ�ļ�ͷ
				getline(infile, line);
			}
		}
		stringstream week, sow, B, L, H, VE, VN, VU, E_std, N_std, U_std, VE_std, VN_std, VU_std;
		gnssrawdata->Gnssalldata.epoch = epoch;
		week    <<	line.substr(5, 4);
		sow     <<	line.substr(12, 10);
		B       <<	line.substr(26, 14);
		L       <<	line.substr(44, 14);
		H		<<	line.substr(68, 8);
		VE		<<	line.substr(80, 8);
		VN		<<	line.substr(92, 8);
		VU		<<	line.substr(104, 8);
		E_std	<<	line.substr(120, 8);
		N_std	<<	line.substr(135, 8);
		U_std	<<	line.substr(150, 8);
		VE_std	<<	line.substr(162, 8);
		VN_std	<<	line.substr(174, 8);
		VU_std	<<	line.substr(186, 8);
		week	>>	gnssrawdata->Gnssalldata.GPST(epoch, 0);
		sow		>>  gnssrawdata->Gnssalldata.GPST(epoch, 1);
		B		>>	gnssrawdata->Gnssalldata.Pos(epoch, 0);
		L		>>  gnssrawdata->Gnssalldata.Pos(epoch, 1);
		H		>>  gnssrawdata->Gnssalldata.Pos(epoch, 2);
		VE		>>  gnssrawdata->Gnssalldata.Vel(epoch, 0);
		VN	    >>  gnssrawdata->Gnssalldata.Vel(epoch, 1);
		VU		>>  gnssrawdata->Gnssalldata.Vel(epoch, 2);
		E_std	>>  gnssrawdata->Gnssalldata.Pos_std(epoch, 0);
		N_std	>>  gnssrawdata->Gnssalldata.Pos_std(epoch, 1);
		U_std	>>  gnssrawdata->Gnssalldata.Pos_std(epoch, 2);
		VE_std	>>  gnssrawdata->Gnssalldata.V_std(epoch, 0);
		VN_std	>>  gnssrawdata->Gnssalldata.V_std(epoch, 1);
		VU_std	>>  gnssrawdata->Gnssalldata.V_std(epoch, 2);
		gnssrawdata->readhead = true;
		epoch++;

	}

}

/*******************************************************
* @brief       ���˲�����ʱ��ȡ
* @author      Sihan Wang
* @paras[in]   GNSSRAWDATA�ṹ�壬�������õ�������
* @paras[out]  none
* @return      void
* @Date        25/5/2022
*******************************************************/
void getGNSSres(int epoch_index, GNSSRAWDATA* gnssrawdata)
{
	gnssrawdata->Gnssresult.epoch_index = epoch_index;//���ǽ����Ԫ���
	if (epoch_index > 1)
	{
		gnssrawdata->Gnssresult.sampling_interval = gnssrawdata->Gnssalldata.GPST(epoch_index, 1) - gnssrawdata->Gnssalldata.GPST(epoch_index - 1, 1);//����һ��Ԫ��GNSS�۲�ʱ��ʱ�������׸���Ԫ��1��
	}
	else
	{
		gnssrawdata->Gnssresult.sampling_interval = 1.0;
	}
	gnssrawdata->Gnssresult.tow = gnssrawdata->Gnssalldata.GPST(epoch_index, 1);//GPS����
	/*λ��*/
	gnssrawdata->Gnssresult.gnss_pos(0) = gnssrawdata->Gnssalldata.Pos(epoch_index, 0) / 180.0 * PI;   //γ��(rad)
	gnssrawdata->Gnssresult.gnss_pos(1) = gnssrawdata->Gnssalldata.Pos(epoch_index, 1) / 180.0 * PI;   //����(rad)
	gnssrawdata->Gnssresult.gnss_pos(2) = gnssrawdata->Gnssalldata.Pos(epoch_index, 2);                //�߳�
	/*�ٶ�*/
	//ע��posmind����ļ�Ϊ������
	gnssrawdata->Gnssresult.gnss_vel(0) = gnssrawdata->Gnssalldata.Vel(epoch_index, 1);       //�����ٶ�
	gnssrawdata->Gnssresult.gnss_vel(1) = gnssrawdata->Gnssalldata.Vel(epoch_index, 0);       //�����ٶ�
	gnssrawdata->Gnssresult.gnss_vel(2) = -1.0 * gnssrawdata->Gnssalldata.Vel(epoch_index, 2);//�����ٶ�
	/*λ�÷�����*/
	//ע��posmind����ļ�Ϊ������λ��������׼��
	double e_std = gnssrawdata->Gnssalldata.Pos_std(epoch_index, 0);
	double n_std = gnssrawdata->Gnssalldata.Pos_std(epoch_index, 1);
	double u_std = gnssrawdata->Gnssalldata.Pos_std(epoch_index, 2);
	Matrix3d ned_Dxx = Matrix3d::Zero();
	ned_Dxx(0, 0) = n_std * n_std;
	ned_Dxx(1, 1) = e_std * e_std;
	ned_Dxx(2, 2) = u_std * u_std;
	Matrix3d _DR_inv = DR_inv(gnssrawdata->Gnssresult.gnss_pos);
	//gnssrawdata->Gnssresult.Dxx = _DR_inv * ned_Dxx * _DR_inv.transpose();
	gnssrawdata->Gnssresult.Dxx = ned_Dxx;
	/*�ٶȷ�����*/
	double ve_std = gnssrawdata->Gnssalldata.V_std(epoch_index, 0);
	double vn_std = gnssrawdata->Gnssalldata.V_std(epoch_index, 1);
	double vu_std = gnssrawdata->Gnssalldata.V_std(epoch_index, 2);
	Matrix3d ned_Dvv = Matrix3d::Zero();
	ned_Dvv(0, 0) = vn_std * vn_std;
	ned_Dvv(1, 1) = ve_std * ve_std;
	ned_Dvv(2, 2) = vu_std * vu_std;
	gnssrawdata->Gnssresult.Dvv =  ned_Dvv;
}