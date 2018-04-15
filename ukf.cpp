#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/********************************************
 * UKF Project 2
 * Paul H. Comitz
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


  // Process noise standard deviation longitudinal acceleration in m/s^2
  // std_a_ = 30;
  std_a_ = 0.5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  // std_yawdd_ = 30;
  std_yawdd_ = 0.5;

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
  std::cout<<"phc: In ProcessMeasurement()"<<std::endl;

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
        //time_us_ = meas_package.raw_measurements_[3]/1000000;

        //initialize P_ for radar, Process Covariance matrix
        P_  << 1,0,0,0,0,
         0,std_radr_*std_radr_,0,0,0,
         0,0,1,0,0,
         0,0,0,std_radphi_,0,
         0,0,0,0,std_radphi_;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
       std::cout<<"phc: ProcessMeasurement LASER "<<std::endl;
       // read px and py from measurements, initialize v, psi, psidot =0
      x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;

      //get the timestamp for Lidar units are micro seconds, express in seconds

      time_us_ = meas_package.timestamp_;

      //initialize P_ for Lidar, Process Covariance matrix
      P_  << 1,0,0,0,0,
         0,1,0,0,0,
         0,0,1,0,0,
         0,0,0,1,0,
         0,0,0,0,1;
    }

    //initialize weights - initialized to a vector of size 15 in ctor
    time_us_ = meas_package.timestamp_;

    //std::cout<<"initializing weights"<<std::endl;
    weights_(0) = lambda_ / (lambda_ + n_aug_);
    for (int i = 1; i < weights_.size(); i++) {
          weights_(i) = 0.5 / (n_aug_ + lambda_);
    }
    //filer is initialized
    is_initialized_ = true;
    std::cout<<"phc: initialized"<<std::endl;
    return;
  } //is_initialized_


  //long long is a 64 bit int
  //time_us_= (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;
  // results wildy worng until cast to double ! delta_t wqs 0!
  double delta_t = (double)(meas_package.timestamp_ - time_us_)/1000000;

  std::cout<<"time_us in delta_t calc:"<<time_us_<<std::endl;
  std::cout<<"delta_t at calculation is:"<<delta_t<<std::endl;
  std::cout<<"measure_package.timestamp in delta_t calc:"<<meas_package.timestamp_<<std::endl;
  time_us_ = meas_package.timestamp_;
  std::cout<<"next time_us is:"<<time_us_<<std::endl;

  //generate and predict sigma points, predict mean x_and covariance P_
  Prediction(delta_t);

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
      cout << "Radar measurements 0,1: " << meas_package.raw_measurements_[0] << " " << meas_package.raw_measurements_[1] << endl;
      UpdateRadar(meas_package);
    }
  if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
      cout << "Lidar measurements 0,1: " << meas_package.raw_measurements_[0] << " " << meas_package.raw_measurements_[1] << endl;
      UpdateLidar(meas_package);
  }

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
  //phc - use later
  int n_sig = 2 * n_aug_+1;

  std::cout<<"phc: In Prediction()"<<std::endl;

  /**
   * Generate Sigma Points
   */
  //from L7_17
  //create augmented mean vector
  VectorXd x_aug = VectorXd(7);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  // create augmented mean state
  // set first 5 elements of vector x_aug to x
  x_aug.head(5) = x_;
  //mean values of acceleration noises are 0
  x_aug(5) = 0;
  x_aug(6) = 0;

  //create augmented covariance matrix
  P_aug.fill(0.0);

  //set top left of P_aug (7,7) to P (5,5)
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;
  //show P_aug
  std::cout<<"P_aug"<<std::endl<<P_aug<<std::endl;

  //from UDACITY SOLN L7_18
  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i< n_aug_; i++)
  {
    Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }

  //show Xsig_aug
  std::cout<<"phc: Xsig_aug"<<std::endl<<Xsig_aug<<std::endl;

  /**
   * Predict Sigma Points
   */
   //from L7_20
  for (int i = 0; i< 2*n_aug_+1; i++)
  {
    //extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    //write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  } //predict sigma points

  //show predicted sigma points Xsig_aug
  //std::cout<<"phc: Xsig_pred"<<std::endl<<Xsig_pred_<<std::endl;

  /**
   * Predict mean and covariance
   */
  // from L7_24
  // set weights
  //create vector for weights
  VectorXd weights = VectorXd(2*n_aug_+1);
  double weight_0 = lambda_/(lambda_+n_aug_);
  weights(0) = weight_0;
  for (int i=1; i<2*n_aug_+1; i++) {  //2n+1 weights
    double weight = 0.5/(n_aug_+lambda_);
    weights(i) = weight;
  }

   //predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x_ = x_+ weights(i) * Xsig_pred_.col(i);
  }

  //show predicted state mean
  std::cout<<"phc: x_"<<std::endl<<x_<<std::endl;

  //predicted state covariance matrix
  P_.fill(0.0);
  std::cout<<"nsig is:" <<n_sig<<std::endl;
  for (int i = 0; i < n_sig; i++) {  //iterate over sigma points
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    //check yaw in range
    // 4/6/2018 CheckAngle never returns at step 53
    //CheckAngle(&x_diff(3));
    //std::cout<<"checking yaw angle range"<<std::endl;
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;

    //std::cout<<"x_diff(3) is:"<<x_diff(3)<<std::endl;

    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P_ = P_ + weights(i) * x_diff * x_diff.transpose();
  }

  //show predicted state covariance
  std::cout<<"phc: P_"<<std::endl<<P_<<std::endl;

} //Prediction

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

    // 3/20/18 try p1 approach
    // p1 approach does not work

    // lidar has px, py measurements
    int n_z = 2;

    // matrix for Sigma points, measurement space
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
    Zsig.fill(0.0);

    //predicted measurement, mean px, py
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0.0);

    //declare measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z,n_z);
    S.fill(0.0);

    // transform sigma points to measurement space and
    // calculate mean predicted measurement z_pred
    // using L7-25 as a guide
    double p_x,p_y = 0.0;
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 sigma points
        // build transformed points
        p_x = Xsig_pred_(0,i);
        p_y = Xsig_pred_(1,i);
        Zsig.col(i) << p_x,p_y;
        z_pred = z_pred + weights_(i) * Zsig.col(i);
    }

    //Calculate measurement covariance S
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 sigma points
      //residual
      VectorXd z_diff = Zsig.col(i) - z_pred;
      S = S + weights_(i) * z_diff * z_diff.transpose();
    }

    //need R measurement noise covariance matrix
    MatrixXd R_laser = MatrixXd(2, 2);
    R_laser << std_laspx_*std_laspx_, 0,
              0, std_laspy_*std_laspy_;

    //add measurement covariance S and R
    S = S + R_laser;


    /// copied from Radar 4/15
    //need to calculate Cross correlation T, Kalman gain K
    // n_x = 5, n_z = 3 for radar, 2 for Lidar
    MatrixXd Tc = MatrixXd(n_x_,n_z);
    Tc.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 sigma points

        //residual, nothing to normalize, p_x, p_y
        VectorXd z_diff = Zsig.col(i) - z_pred;

        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        //angle normalization, psi
        while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
     }

    //Calculate Kalman gain K
    MatrixXd K = Tc * S.inverse();

    //to update state x, need actual measurement
    //vector for incoming radar measurement
    VectorXd z = VectorXd(n_z);
    p_x = meas_package.raw_measurements_(0);
    p_y = meas_package.raw_measurements_(1);
    z << p_x,p_y;

    //residual
    VectorXd z_diff = z - z_pred;

    //do Normalized Innovation Squared
    double NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;

    //update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K*S*K.transpose();
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
    // UPDATE STEP
    // transform predicted state into measurement space
    // phc 2/25/2018
    // from L7_26_27

    //create matrix for sigma points in measurement space
    int n_z = 3;  //rho, phi, rhodot for radar
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

    //transform sigma points into measurement space
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 sigma points

      // extract values for better readibility
      double p_x = Xsig_pred_(0,i);
      double p_y = Xsig_pred_(1,i);
      double v  = Xsig_pred_(2,i);
      double yaw = Xsig_pred_(3,i);

      double v1 = cos(yaw)*v;
      double v2 = sin(yaw)*v;

      // measurement model
      Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
      Zsig(1,i) = atan2(p_y,p_x);                                 //phi
      Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
    }

    //mean predicted measurement
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0.0);

    for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
    }

    //Measurement Covariance
    MatrixXd S = MatrixXd(n_z,n_z);
    S.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 sigma points
      //residual
      VectorXd z_diff = Zsig.col(i) - z_pred;

      //angle normalization
      while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
      while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

      S = S + weights_(i) * z_diff * z_diff.transpose();
    }

    //add measurement noise covariance matrix
    MatrixXd R = MatrixXd(n_z,n_z);
    R <<    std_radr_*std_radr_, 0, 0,
            0, std_radphi_*std_radphi_, 0,
            0, 0,std_radrd_*std_radrd_;
    S = S + R;

    std::cout<<"S is:"<<S<<std::endl;
    std::cout<<"z_pred is:"<<z_pred<<endl;

    // 4/5/2018 11:56 PM
    //need to calculate Cross correlation T, kalman gain K
    // n_x = 5, n_z = 3
    MatrixXd Tc = MatrixXd(n_x_,n_z);
    Tc.fill(0.0);

    //could fold into above, but this is simpler for now
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        //angle normalization
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        //angle normalization
        while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
     }

    //Kalman gain K
    MatrixXd K = Tc * S.inverse();

    //need actual measurement
    //vector for incoming radar measurement
    VectorXd z = VectorXd(n_z);
    double meas_rho = meas_package.raw_measurements_(0);
    double meas_phi = meas_package.raw_measurements_(1);
    double meas_rhod = meas_package.raw_measurements_(2);
    z << meas_rho, meas_phi,meas_rhod;

    //residual
    VectorXd z_diff = z - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    //do Normalized Innovation Squared
    double NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;

    //update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K*S*K.transpose();



} //UpdateRadar

/**
 * phc
 * convenience method for checking if an angle is
 * between M_PI and -M_PI
 */
void UKF::CheckAngle(double* ang) {
  std::cout<<"in CheckAngle"<<std::endl;
  while(*ang > M_PI)
    *ang -= 2 * M_PI;

  while(*ang < -M_PI)
      *ang += 2 * M_PI;
}
