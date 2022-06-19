#pragma once
#ifndef _CONFIG_H_
#define _CONFIG_H_
#include"ConstNums.h"

#define   BaseB                  30.528441628       // ��վγ��
#define   BaseL                  114.356976890      // ��վ����
#define   BaseH                  41.2160            // ��վ�߳�


#define   IMU_SAMPLE_HZ          200.0              // ����Ƶ��
#define   IMU_SAMPLE_INTERVAL    1/IMU_SAMPLE_HZ    // �������
#define   InitEpochs             60000

#define   lever_arm_forward      -0.250             //  m, GNSS to IMU
#define   lever_arm_right        -0.225             //  m, GNSS to IMU
#define   lever_arm_down         -0.995             //  m, GNSS to IMU
//#define   lever_arm_forward      -0.068             //  m, GNSS to IMU
//#define   lever_arm_right        0.2965             //  m, GNSS to IMU
//#define   lever_arm_down         -0.922             //  m, GNSS to IMU

// system noise configuration
#define   sigma_sa               1e-5               // ���ٶȼƶ�̬�������ӱ�׼��
#define   sigma_sg               1e-5               // �����Ƕ�̬�������ӱ�׼��
#define   sigma_ba               1.4E-5             // ���ٶȼƶ�̬��ƫ��׼��
#define   sigma_bg               7 * D2R /3600      // ���ݶ�̬��ƫ��׼��

// һ�׸�˹����ɷ�������ʱ��
#define   rt_gb					 3600.0              // ������ƫ��������ɷ�������ʱ��
#define   rt_ab					 3600.0              // ���ٶȼ���ƫ��������ɷ�������ʱ��
#define   rt_gs					 3600.0              // ���ݱ���������������ɷ�������ʱ��
#define   rt_as					 3600.0              // ���ٶȼƱ���������������ɷ�������ʱ��
#define   ARW                    0.15 * D2R / 60.0
#define   VRW                    0.57 / 60.0

//init variance configuration
#define   deltapos_flat          4.9e-9             // ��γ�ȷ���λ������
#define   deltapos_vertical      1.0                // ��ֱ����λ������
#define   deltavel_flat          0.01               // ˮƽ�����ٶ�����
#define   deltavel_vertical      0.01               // ��ֱ�����ٶ�����
#define   delta_pitch            1e-6               // ��������
#define   delta_roll             1e-6               // ���������
#define   delta_yaw              0.01               // ���������
#define   delta_acc_b            1.6e-6             // ���ٶȼ���ƫ����
#define   delta_gyro_b           1.6e-5             // ������ƫ����
#define   delta_acc_s            1e-6               // ���ٶȼƱ�����������
#define   delta_gyro_s           1e-6               // �����Ǳ�����������
#endif