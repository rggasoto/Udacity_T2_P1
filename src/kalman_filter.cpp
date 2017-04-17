#include "kalman_filter.h"
#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */

	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
	//std::cout << "update y" << std::endl;
	VectorXd y = z - Tools::h(x_);
	//std::cout << "Ht" << std::endl;
	MatrixXd Ht = Hj_.transpose();
	//std::cout << "S" << std::endl;
	MatrixXd S = Hj_ * P_ * Ht + R_;
	//std::cout << "Si" << std::endl;
	MatrixXd Si = S.inverse();
	//std::cout << "PHt" << std::endl;
	MatrixXd PHt = P_ * Ht;
	//std::cout << "K" << std::endl;
	MatrixXd K = PHt * Si;

	//new estimate
	//std::cout << "New Estimate" << std::endl;
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	//std::cout << "P" << x_size<<std::endl;
	P_ = (I - K * Hj_) * P_;
	//std::cout << "Done" << std::endl;

}
