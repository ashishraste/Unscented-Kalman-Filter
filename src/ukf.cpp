#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.3;
  
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
  // State covariance matrix
  P_.fill(0.);
  P_(0,0) = 0.15;
  P_(1,1) = 0.15;
  P_(2,2) = 1.;
  P_(3,3) = 1.;
  P_(4,4) = 1.;

  // State dimension
  n_x_ = 5;
  // Augmented state dimension
  n_aug_ = 7;
  // Design parameter for sigma points
  lambda_ = 3 - n_x_;
  // Time when the state is true
  time_us_ = 0;

  // Predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_ + 1);

  // Set the weights for sigma points
  weights_ = VectorXd(2*n_aug_ + 1);
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int i=1; i<=2*n_aug_; ++i)
    weights_(i) = 1 / (2 * (lambda_ + n_aug_));

  // Consistency check measure for Lidar measurement corrections
  nis_laser_ = 0.;
  // Consistency check measure for Radar measurement corrections
  nis_radar_ = 0.;

  is_initialized_ = false;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {

  // Run the filter only when one of Radar or Laser measurements are available.
  if ((meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) ||
    (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_)) {

    if (!is_initialized_) {
      /**
       * Initialize state.
       * State contains the following variables of an object.
       1. x position
       2. y position
       3. velocity
       4. yaw angle
       5. yaw rate
       */
      cout << "UKF: " << endl;
      if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
        /**
        Convert radar from polar to cartesian coordinates and initialize state.
        */
        float ro = meas_package.raw_measurements_[0];
        float theta = meas_package.raw_measurements_[1];

        x_ << ro * cos(theta), ro * sin(theta), 0., 0., 0.;
      }
      else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
        x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0., 0., 0.;
      }
      is_initialized_ = true;
      time_us_ = meas_package.timestamp_;
      return;
    }

    // Compute the time elapsed between the current and previous measurements
    float dt = (meas_package.timestamp_ - time_us_) / 1000000.0;  //dt - expressed in seconds
    time_us_ = meas_package.timestamp_;

    // Prediction routine
    if (dt > 0.0001) {
      Prediction(dt);
    }

    // Update Routine
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      UpdateRadar(meas_package);
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      UpdateLidar(meas_package);
    }
  }

}

const MatrixXd UKF::GenerateSigmaPoints() {
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  // Create augmented mean state
  VectorXd x_aug = VectorXd(7);
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  // Create augmented covariance matrix
  MatrixXd P_aug = MatrixXd(7, 7);
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_ * std_a_;
  P_aug(6,6) = std_yawdd_ * std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  double lambda_plus_naug_sqrt = sqrt(lambda_ + n_aug_);
  for (int i = 0; i< n_aug_; i++)
  {
    Xsig_aug.col(i+1) = x_aug + lambda_plus_naug_sqrt * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - lambda_plus_naug_sqrt * L.col(i);
  }
  return Xsig_aug;
}

void UKF::PredictSigmaPoints(const MatrixXd & Xsig_aug, const double & delta_t) {
  Xsig_pred_.fill(0.);
  for (int i=0; i<2*n_aug_+1; ++i) {
    // Assign state variables for further calculations
    double px = Xsig_aug(0,i);
    double py = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yaw_d = Xsig_aug(4,i);
    double nu_acc = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    // Add noise and assign to predicted sigma points.
    // Note: We aren't predicting noise-elements (acc and yaw-rate)
    double dt2 = delta_t * delta_t;
    if (yaw_d == 0.) {
      Xsig_pred_(0,i) = px + v * cos(yaw) * delta_t + 0.5f * nu_acc * dt2 * cos(yaw);
      Xsig_pred_(1,i) = py + v * sin(yaw) * delta_t + 0.5f * nu_acc * dt2 * sin(yaw);
    }
    else {
      Xsig_pred_(0,i) = px + (v / yaw_d) * (sin(yaw + yaw_d * delta_t) - sin(yaw)) + 0.5f * nu_acc * dt2 * cos(yaw);
      Xsig_pred_(1,i) = py + (v / yaw_d) * (-cos(yaw + yaw_d * delta_t) + cos(yaw)) + 0.5f * nu_acc * dt2 * sin(yaw);
    }
    Xsig_pred_(2,i) = v + nu_acc * delta_t;
    Xsig_pred_(3,i) = yaw + yaw_d + 0.5 * nu_yawdd * dt2;
    Xsig_pred_(4,i) = yaw_d + nu_yawdd * delta_t;
  }
}

void UKF::PredictMeanAndCovariance() {
  // Predicted state
  x_.fill(0.);
  for (int i=0; i<2*n_aug_+1; ++i) {
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  // Predicted covariance
  P_.fill(0.);
  for (int i=0; i<2*n_aug_+1; ++i) {
    // State difference for a given sigma point.
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    // Angle normalization
    while (x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;
    while (x_diff(3) > M_PI) x_diff(3) -= 2. * M_PI;

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  //// Generate Sigma points
  MatrixXd Xsig_aug;
  Xsig_aug = GenerateSigmaPoints();

  //// Predict Sigma points
  PredictSigmaPoints(Xsig_aug, delta_t);

  //// Predict mean and covariance of the state using predicted Sigma points.
  PredictMeanAndCovariance();
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {

  // Extract measurement vector; 2-dimensional
  VectorXd z = meas_package.raw_measurements_;
  int n_z = 2;

  // Matrix for holding predicted sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_ + 1);

  // Predicted measurement and its covariance from using predicted sigma points
  VectorXd z_pred = VectorXd(n_z);
  MatrixXd S = MatrixXd(n_z, n_z);

  // Transform predicted sigma points into measurement space
  for (int i=0; i<2*n_aug_+1; ++i) {
    // Predicted state variables relevant to laser measurement
    double px = Xsig_pred_(0,i);
    double py = Xsig_pred_(1,i);

    // Measurement model
    Zsig(0,i) = px;
    Zsig(1,i) = py;
  }

  // Calculate mean predicted measurement
  z_pred.fill(0.);
  for (int i=0; i<2*n_aug_+1; ++i) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  // Calculate measurement covariance
  S.fill(0.);
  for (int i=0; i<2*n_aug_+1; ++i) {
    // Residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  // Add measurement noise to S
  MatrixXd R = MatrixXd(n_z, n_z);
  R.fill(0.);
  R(0,0) = std_laspx_ * std_laspx_;
  R(1,1) = std_laspy_ * std_laspy_;
  S = S + R;

  //// Update state
  // Calculate cross-correlation between sigma points in state space and measurement space
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.);
  for (int i=0; i<2*n_aug_+1; ++i) {
    // Residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // State difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain
  MatrixXd K = Tc * S.inverse();

  // Measurement residual
  VectorXd y = z - z_pred;

  // Update state and its covariance
  x_ = x_ + K * y;
  P_ = P_ - K * S * K.transpose();

  // Calculate NIS for laser measurement
  nis_laser_ = y.transpose() * S.inverse() * y;

}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  // Extract measurement vector; 3-dimensional
  VectorXd z = meas_package.raw_measurements_;
  int n_z = 3;

  // Matrix for holding predicted sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_ + 1);

  // Predicted measurement and its covariance from using predicted sigma points
  VectorXd z_pred = VectorXd(n_z);
  MatrixXd S = MatrixXd(n_z, n_z);

  // Transform predicted sigma points into measurement space
  for (int i=0; i<2*n_aug_+1; ++i) {
    // Predicted state variables relevant to Radar measurement
    double px = Xsig_pred_(0,i);
    double py = Xsig_pred_(1,i);
    double v = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    // Measurement model
    Zsig(0,i) = sqrt(px*px + py*py);
    Zsig(1,i) = atan2(py, px);
    Zsig(2,i) = v * (px * cos(yaw) + py * sin(yaw)) / Zsig(0,i);
  }

  // Calculate mean predicted measurement
  for (int i=0; i<2*n_aug_+1; ++i) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  // Calculate measurement covariance matrix S
  for (int i=0; i<2*n_aug_+1; ++i) {
    // Residual
    VectorXd z_err = Zsig.col(i) - z_pred;

    // Angle normalization
    while (z_err(1) > M_PI) z_err(1) -= 2*M_PI;
    while (z_err(1) < -M_PI) z_err(1) += 2*M_PI;

    S = S + weights_(i) * z_err * z_err.transpose();
  }

  // Add measurement noise to S
  MatrixXd R = MatrixXd(n_z, n_z);
  R.fill(0.);
  R(0,0) = std_radr_ * std_radr_;
  R(1,1) = std_radphi_ * std_radphi_;
  R(2,2) = std_radrd_ * std_radrd_;
  S = S + R;

  //// Update state

  // Calculate cross-correlation between sigma points in state space and measurement space
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.);
  for (int i=0; i<2*n_aug_+1; ++i) {
    // Residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // State difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    // Angle normalization in state and measurement
    while (x_diff(3) > M_PI) x_diff(3) -= 2.*M_PI;
    while (x_diff(3) < -M_PI) x_diff(3) += 2.*M_PI;

    while (z_diff(1) > M_PI) z_diff(1) -= 2.*M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain
  MatrixXd K = Tc * S.inverse();

  // Measurement residual
  VectorXd y = z - z_pred;

  // Update state and its covariance
  x_ = x_ + K * y;
  P_ = P_ - K * S * K.transpose();

  // Calculate NIS for laser measurement
  nis_radar_ = y.transpose() * S.inverse() * y;
}
