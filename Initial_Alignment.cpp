#include"Initial_Allignment.h"

using namespace Eigen;

/*******************************************************
* set accel and gyro data by using static alignment raw data
* @author      Sihan Wang
* @paras       gravity(m/s^2) and lat(units:rad)
* @return      void
*******************************************************/
void InitAlign::set(const Eigen::MatrixXd& accel, const Eigen::MatrixXd& gyro, double gravity, double lat)
{
	this->_accel   =  accel;
	this->_gyro    =  gyro;
	this->_gravity =  gravity;
	this->_lat     =  lat;
}

/*******************************************************
* use analysis method to <coarse init-align>
* @author      Sihan Wang
* @paras
* @return rotation matrix C_b^n, means from the sensor frame to nav frame
*******************************************************/
Eigen::Matrix3d InitAlign::initAlign()
{
	// rotation matrix C_b^n, means from the sensor frame to nav frame
	// 这里的坐标系是NED
	Matrix3d rotmat_cbn;
	// Calculate the mean vector of accel and gyros 
	// construct the orthogonal vector v_construct
	Vector3d accel_average = _accel.colwise().mean();
	Vector3d gyro_average = _gyro.colwise().mean();
	Vector3d v_construct = accel_average.cross(gyro_average);

	Vector3d accel_n, gyro_n, v_n;
	accel_n << 0, 0, -(this->_gravity);
	gyro_n << OMEGA * cos(this->_lat), 0, -OMEGA * sin(this->_lat);
	v_n = accel_n.cross(gyro_n);

	Matrix3d n_mat, b_mat;	//constructed matrix for alignment in nav frame and sensor frame
	n_mat << accel_n.transpose(),
			 gyro_n.transpose(),
			 v_n.transpose();
	b_mat << accel_average.transpose(),
			 gyro_average.transpose(),
			 v_construct.transpose();

	rotmat_cbn = n_mat.inverse() * b_mat;

	//iteration to normalize the rotation matrix C_b^n
	for (int i = 0; i < 5; i++)
	{
		rotmat_cbn = (rotmat_cbn + rotmat_cbn.transpose().inverse()) * 0.5;
	}
	return rotmat_cbn;
}
