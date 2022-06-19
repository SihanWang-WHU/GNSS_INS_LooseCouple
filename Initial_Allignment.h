#pragma once

#ifndef INITIAL_ALIGNMENT_H
#define INITIAL_ALIGNMENT_H

#include<Eigen/Dense>
#include"ConstNums.h"


//   @brief            initial alignment class which uses the analysis coarse method.
//   @parameters [in]  raw_accel,raw_gyro,whose size is (epoch_num,3),type is Eigen::MatrixXd
//   @parameters [out] init_rot_mat, the rot matrix obtained by init-align,whose type is Eigen::Matrix3d
class InitAlign {
public: 
    //functions
    // set accel and gyro data by using static alignment raw data
    void set(const Eigen::MatrixXd& accel, const Eigen::MatrixXd& gyro, double gravity, double lat);
    // use analysis method to <coarse init-align>
    Eigen::Matrix3d initAlign();

private:
    Eigen::MatrixXd _accel; // static raw accel&gyro data, use for analysis coarse alignment 
    Eigen::MatrixXd _gyro;  // should be MatrixXd(n,3)
    double _gravity;        // gravity acceleration
    double _lat;            // local latitude, units:rad
};


#endif
