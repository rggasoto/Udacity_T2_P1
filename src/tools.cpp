#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if (estimations.size() != ground_truth.size()
		|| estimations.size() == 0) {
		std::cout << "Invalid estimation or ground_truth data" << std::endl;
		return rmse;
	}

	//accumulate squared residuals
	for (unsigned int i = 0; i < estimations.size(); ++i) {

		VectorXd residual = estimations[i] - ground_truth[i];

		//coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	//calculate the mean
	rmse = rmse / estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
	MatrixXd Hj(3, 4);
	//recover state parameters
	long double px = x_state(0);
	long double py = x_state(1);

	
	long double vx = x_state(2);
	long double vy = x_state(3);

	//pre-compute a set of terms to avoid repeated calculation
	long double c1 = px*px + py*py;
	c1 = c1 == 0 ? 1e-12 : c1;
	long double c2 = sqrt(c1);
	long double c3 = (c1*c2);

	

	//compute the Jacobian matrix
	Hj << (px / c2), (py / c2), 0, 0,
		-(py / c1), (px / c1), 0, 0,
		py*(vx*py - vy*px) / c3, px*(px*vy - py*vx) / c3, px / c2, py / c2;

	return Hj;
}

VectorXd Tools::h(const Eigen::VectorXd& x) {
	long double px, py, vx, vy, sqr;
	px = x(0);
	py = x(1);

	px = fabs(px) != 0 ? px : 1e-12;
	py = fabs(py) != 0 ? py : 1e-12;
	vx = x(2);
	vy = x(3);
	sqr = sqrt(pow(px,2) + pow(py,2));
	//sqr = sqr == 0 ? 1e-6 : sqr;
	VectorXd h = VectorXd(3);
	h << sqr,
		atan2(py , px),
		(px*vx + py*vy) / sqr;

	return h;
}
