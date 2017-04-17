#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
	is_initialized_ = false;

	previous_timestamp_ = 0;

	// initializing matrices
	R_laser_ = MatrixXd(2, 2);
	R_radar_ = MatrixXd(3, 3);
	H_laser_ = MatrixXd(2, 4);

	//measurement covariance matrix - laser
	R_laser_ << 0.0225, 0,
		0, 0.0225;

	//measurement covariance matrix - radar
	R_radar_ << 0.09, 0, 0,
		0, 0.0009, 0,
		0, 0, 0.09;

	/**
	TODO:
	  * Finish initializing the FusionEKF.
	  * Set the process and measurement noises
	*/
	//create a 4D state vector, we don't know yet the values of the x state
	ekf_.x_ = VectorXd(4);

	//state covariance matrix P
	ekf_.P_ = MatrixXd(4, 4);
	ekf_.P_ << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1000, 0,
		0, 0, 0, 1000;


	//measurement covariance
	ekf_.R_ = MatrixXd(2, 2);

	//measurement matrix	

	ekf_.H_ = MatrixXd(2, 4);
	ekf_.H_ << 1, 0, 0, 0,
		0, 1, 0, 0;

	// initial Radar jacobian H
	ekf_.Hj_ = MatrixXd(3, 4);


	//the initial transition matrix F_
	ekf_.F_ = MatrixXd(4, 4);
	ekf_.F_ << 1, 0, 1, 0,
		0, 1, 0, 1,
		0, 0, 1, 0,
		0, 0, 0, 1;

	ekf_.Q_ = MatrixXd(4, 4);
	//set the acceleration noise components
	noise_ax = 9;
	noise_ay = 9;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


	/*****************************************************************************
	 *  Initialization
	 ****************************************************************************/
	if (!is_initialized_) {		
		// first measurement
		//cout << "EKF: " << endl;
		ekf_.x_ = VectorXd(4);
		ekf_.x_ << 1, 1, 1, 1;
		//cout << "x_ = " << measurement_pack.sensor_type_ << endl;
		switch (measurement_pack.sensor_type_) {
		case MeasurementPackage::RADAR:
			/**
			Convert radar from polar to cartesian coordinates and initialize state.
			*/
			long double p, phi, pdot;
			p = measurement_pack.raw_measurements_[0];
			phi = measurement_pack.raw_measurements_[1];
			pdot = measurement_pack.raw_measurements_[2];
			ekf_.x_ << p*cos(phi), p*sin(phi), pdot*cos(phi), pdot*sin(phi);
			break;
		case MeasurementPackage::LASER:
		default:

			/**
			Initialize state.
			*/
			//cout << "LIDAR= " << ekf_.x_ << endl;
			ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;

			break;

		}
		// done initializing, no need to predict or update
		is_initialized_ = true;
		previous_timestamp_ = measurement_pack.timestamp_;	
		cout << "x_ = " << ekf_.x_ << endl;
		cout << "P_ = " << ekf_.P_ << endl;
		return;
	}

	/*****************************************************************************
	 *  Prediction
	 ****************************************************************************/

	
	double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
	
	
	
	previous_timestamp_ = measurement_pack.timestamp_;
	VectorXd z;

	ekf_.F_(0, 2) = dt;
	ekf_.F_(1, 3) = dt;

	double dt4, dt3, dt2;
	dt2 = dt*dt;
	dt3 = dt2*dt;
	dt4 = dt3*dt;
	dt4 /= 4.0;
	dt3 /= 2.0;	
	ekf_.Q_ << dt4*noise_ax, 0, dt3*noise_ax, 0,
		0, dt4*noise_ay, 0, dt3*noise_ay,
		dt3*noise_ax, 0, dt2*noise_ax, 0,
		0, dt3*noise_ay, 0, dt2*noise_ay;

	
	if(dt) ekf_.Predict();

	/*****************************************************************************
	 *  Update
	 ****************************************************************************/

	
	switch (measurement_pack.sensor_type_) {
	case MeasurementPackage::RADAR:
		z = VectorXd(3);
		ekf_.R_ = R_radar_;
		// Radar updates
		ekf_.Hj_ = Tools::CalculateJacobian(ekf_.x_);

		ekf_.UpdateEKF(measurement_pack.raw_measurements_);

		break;
	case MeasurementPackage::LASER:
	default:
		z = VectorXd(4);
		ekf_.R_ = R_laser_;
		// Laser updates
		ekf_.Update(measurement_pack.raw_measurements_);

		break;

	}
	
	// print the output

	cout << "x_ = " << ekf_.x_ << endl;
	cout << "P_ = " << ekf_.P_ << endl;
}
