#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/********************************************
 * UKF Project 2
 ********************************************/

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 * Project 2 UKF
 */
UKF::UKF() {

  std::cout <<"phc: in UKF ctor:"<<std::endl;
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  P_  << 1,0,0,0,0,
         0,1,0,0,0,
         0,0,1,0,0,
         0,0,0,1,0,
         0,0,0,0,1;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;

  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  //phc
  n_x_ = 5;
  n_aug_=7;
  //also need Xsig?
  Xsig_pred_= MatrixXd(n_x_, 2*n_aug_+1);
  weights_ = VectorXd(2*n_aug_+1);
  lambda_ = 3 - n_x_;
  time_us_= 0;

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  std::cout<<"phc: ProcessMeasurement"<<std::endl;

  if (!is_initialized_) {
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
         std::cout<<"phc: ProcessMeasurement RADAR"<<std::endl;
        // Convert radar from polar to cartesian coordinates and initialize state.
        float rho = meas_package.raw_measurements_[0]; // range
        float phi = meas_package.raw_measurements_[1]; // bearing
        float rho_dot = meas_package.raw_measurements_[2]; // velocity of rho
        // polar to cartesian
        float px = rho * cos(phi);
        float py = rho * sin(phi);
        float vx = rho_dot * cos(phi);
        float vy = rho_dot * sin(phi);
        float v  = sqrt(vx * vx + vy * vy);
        x_ << px, py, v, 0, 0;

        //get the timestamp for radar units are micro seconds, express in seconds
        time_us_ = meas_package.raw_measurements_[3]/1000000;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
       std::cout<<"phc: ProcessMeasurement LASER "<<std::endl;
       // read px and py from measurements, initialize v, psi, psidot =0
      x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;

      //get the timestamp for lidar units are micro seconds, express in seconds
      //time_us_ = meas_package.raw_measurements_[2]; - this blew it up ?
      time_us_ = meas_package.timestamp_;
    }


    //initialize weights - initialized to a vector of size 15 in ctor

    std::cout<<"initializing weights"<<std::endl;
    weights_(0) = lambda_ / (lambda_ + n_aug_);
    for (int i = 1; i < weights_.size(); i++) {
          weights_(i) = 0.5 / (n_aug_ + lambda_);
    }

    is_initialized_ = true;
    std::cout<<"phc: initialized"<<std::endl;
    return;
  } //is_initialized_


}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}
