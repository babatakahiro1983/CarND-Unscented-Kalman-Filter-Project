#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {

  is_initialized_ = false;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // time when the state is true, in us
  time_us_ = 0;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.230;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.282;

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

  // Weights of sigma points
  weights_;

  // initial State dimension
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = 7;

  // initial Sigma point spreading parameter
  lambda_ = 3 - n_x_;

  // initial predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  // initial  Radar data counter
  radar_counter_ = 0;

  // initial  Radar NIS threshold
  radar_NIS_threshold_ = 7.815;

  // initial  Radar more NIS counter
  radar_more_NIS_counter_ = 0;

  // initial  Radar more NIS rate
  radar_more_NIS_rate_ = 0;

  // initial  Lasar data counter
  laser_counter_ = 0;

  // initial  Lasar NIS threshold
  laser_NIS_threshold_ = 5.991;

  // initial  Lasar more NIS counter
  laser_more_NIS_counter_ = 0;

  // initial  Lasar more NIS rate
  laser_more_NIS_rate_ = 0;

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
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

  /*****************************************************************************
  *  Initialization
  ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
    * Initialize the state ukf_.x_ with the first measurement.
    * Create the covariance matrix.
    * Remember: you'll need to convert radar from polar to cartesian
    coordinates.
    */
    // first measurement
    cout << "UKF: " << endl;
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float ro = meas_package.raw_measurements_[0];
      float theta = meas_package.raw_measurements_[1];
      float ro_dot = meas_package.raw_measurements_[2];
      x_ << ro * std::cos(theta), ro * std::sin(theta), 0, 0, 0;
    } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      // set the state with the initial location and zero velocity
      x_ << meas_package.raw_measurements_[0],
          meas_package.raw_measurements_[1], 0, 0, 0;
    }
    P_ << 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
        0, 1;
    // done initializing, no need to predict or update
    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
  *  Prediction
  ****************************************************************************/
  // compute the time elapsed between the current and previous measurements
  float dt = (meas_package.timestamp_ - time_us_) /
             1000000.0; // dt - expressed in seconds
  time_us_ = meas_package.timestamp_;
  Prediction(dt);
  /*****************************************************************************
  *  Update
  ****************************************************************************/
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    UpdateRadar(meas_package);
  } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    // Laser updates
    UpdateLidar(meas_package);
  }
  // print the output
  cout << "x_ = " << x_ << endl;
  cout << "P_ = " << P_ << endl;
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

  MatrixXd Xsig;
  GenerateSigmaPoints(n_x_, P_, lambda_, x_, &Xsig);
  MatrixXd Xsig_aug;
  AugmentedSigmaPoints(n_aug_, x_, P_, std_a_, std_yawdd_, lambda_, &Xsig_aug);
  SigmaPointPrediction(n_aug_, Xsig_aug, delta_t, &Xsig_pred_);
  PredictMeanAndCovariance(weights_, n_aug_, lambda_, Xsig_pred_, &x_, &P_);
}

void UKF::GenerateSigmaPoints(const int &n_x_, const MatrixXd &P_,
                              const double &lambda_, const VectorXd &x_,
                              MatrixXd *Xsig_out) {
  // create sigma point matrix
  MatrixXd Xsig = MatrixXd(n_x_, 2 * n_x_ + 1);
  // calculate square root of P
  MatrixXd A = P_.llt().matrixL();
  // set first column of sigma point matrix
  Xsig.col(0) = x_;
  // set remaining sigma points
  for (int i = 0; i < n_x_; i++) {
    Xsig.col(i + 1) = x_ + sqrt(lambda_ + n_x_) * A.col(i);
    Xsig.col(i + 1 + n_x_) = x_ - sqrt(lambda_ + n_x_) * A.col(i);
  }
  *Xsig_out = Xsig;
}

void UKF::AugmentedSigmaPoints(const int &n_aug_, const VectorXd &x_,
                               const MatrixXd &P_, const double &std_a_,
                               const double &std_yawdd_, const double &lambda_,
                               MatrixXd *Xsig_aug_out) {
  // create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);
  // create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  // create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  // create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;
  // create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5, 5) = P_;
  P_aug(5, 5) = std_a_ * std_a_;
  P_aug(6, 6) = std_yawdd_ * std_yawdd_;
  // create square root matrix
  MatrixXd L = P_aug.llt().matrixL();
  // create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  for (int i = 0; i < n_aug_; i++) {
    Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
  }
  *Xsig_aug_out = Xsig_aug;
}

void UKF::SigmaPointPrediction(const int &n_aug_, const MatrixXd &Xsig_aug,
                               const double &delta_t, MatrixXd *Xsig_pred_out) {
  // create matrix with predicted sigma points as columns
  // predict sigma points
  MatrixXd Xsig_pred = MatrixXd(5, 2 * n_aug_ + 1);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    // extract values for better readability
    double p_x = Xsig_aug(0, i);
    double p_y = Xsig_aug(1, i);
    double v = Xsig_aug(2, i);
    double yaw = Xsig_aug(3, i);
    double yawd = Xsig_aug(4, i);
    double nu_a = Xsig_aug(5, i);
    double nu_yawdd = Xsig_aug(6, i);
    // predicted state values
    double px_p, py_p;
    // avoid division by zero
    if (fabs(yawd) > 0.001) {
      px_p = p_x + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
      py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
    } else {
      px_p = p_x + v * delta_t * cos(yaw);
      py_p = p_y + v * delta_t * sin(yaw);
    }
    double v_p = v;
    double yaw_p = yaw + yawd * delta_t;
    double yawd_p = yawd;
    // add noise
    px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
    py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
    v_p = v_p + nu_a * delta_t;
    yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
    yawd_p = yawd_p + nu_yawdd * delta_t;
    // write predicted sigma point into right column
    Xsig_pred(0, i) = px_p;
    Xsig_pred(1, i) = py_p;
    Xsig_pred(2, i) = v_p;
    Xsig_pred(3, i) = yaw_p;
    Xsig_pred(4, i) = yawd_p;
  }
  *Xsig_pred_out = Xsig_pred;
}

void UKF::PredictMeanAndCovariance(VectorXd &weights_, const int &n_aug_,
                                   const double &lambda_, MatrixXd &Xsig_pred_,
                                   VectorXd *x_out, MatrixXd *P_out) {
  // create vector for weights
  weights_ = VectorXd(2 * n_aug_ + 1);
  // set weights
  double weight_0 = lambda_ / (lambda_ + n_aug_);
  weights_(0) = weight_0;
  for (int i = 1; i < 2 * n_aug_ + 1; i++) { // 2n+1 weights
    double weight = 0.5 / (n_aug_ + lambda_);
    weights_(i) = weight;
  }
  // predicted state mean
  VectorXd x_ = VectorXd(5);
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) { // iterate over sigma points
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }
  // predicted state covariance matrix
  MatrixXd P_ = MatrixXd(5, 5);
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) { // iterate over sigma points
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    while (x_diff(3) > M_PI)
      x_diff(3) -= 2. * M_PI;
    while (x_diff(3) < -M_PI)
      x_diff(3) += 2. * M_PI;
    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }
  *x_out = x_;
  *P_out = P_;
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

  int n_z = 2;
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  VectorXd z_pred = VectorXd(n_z);
  MatrixXd S = MatrixXd(n_z, n_z);
  PredictLidarMeasurement(n_aug_, Xsig_pred_, weights_, &Zsig, &z_pred, &S);
  VectorXd z_diff;
  UpdateLidarState(n_z, meas_package, n_x_, Zsig, z_pred, Xsig_pred_, weights_,
                   S, &x_, &P_, &z_diff);
  ComputeNIS(z_diff, S, radar_NIS_threshold_, &NIS_laser_,
             &laser_more_NIS_counter_, &laser_counter_, &laser_more_NIS_rate_);
}

void UKF::PredictLidarMeasurement(const int &n_aug_, const MatrixXd &Xsig_pred_,
                                  VectorXd &weights_, MatrixXd *Zsig_out,
                                  VectorXd *z_out, MatrixXd *S_out) {
  ///////////////  Predict Lidar Measurement /////////////
  // set measurement dimension, lidar can measure x, y
  int n_z = 2;
  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) { // 2n+1 simga points
    // extract values for better readibility
    double p_x = Xsig_pred_(0, i);
    double p_y = Xsig_pred_(1, i);
    double v = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);
    double v1 = cos(yaw) * v;
    double v2 = sin(yaw) * v;
    // measurement model
    Zsig(0, i) = p_x; // x
    Zsig(1, i) = p_y; // y
  }
  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }
  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) { // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    S = S + weights_(i) * z_diff * z_diff.transpose();
  }
  // add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z, n_z);
  R << std_laspx_ * std_laspx_, 0, 0, std_laspy_ * std_laspy_;
  S = S + R;
  *Zsig_out = Zsig;
  *z_out = z_pred;
  *S_out = S;
}

void UKF::UpdateLidarState(const int &n_z,
                           const MeasurementPackage &meas_package,
                           const int &n_x_, MatrixXd &Zsig,
                           const MatrixXd &z_pred, const MatrixXd &Xsig_pred,
                           const VectorXd &weights_, const MatrixXd &S,
                           VectorXd *x_, MatrixXd *P_, VectorXd *z_diff_out) {
  // create vector for incoming radar measurement
  VectorXd z = VectorXd(n_z);
  z = meas_package.raw_measurements_;
  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  // calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) { // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - *x_;
    // angle normalization
    while (x_diff(3) > M_PI)
      x_diff(3) -= 2. * M_PI;
    while (x_diff(3) < -M_PI)
      x_diff(3) += 2. * M_PI;
    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }
  // Kalman gain K;
  MatrixXd K = Tc * S.inverse();
  // residual
  VectorXd z_diff = z - z_pred;
  // update state mean and covariance matrix
  *x_ = *x_ + K * z_diff;
  *P_ = *P_ - K * S * K.transpose();
  *z_diff_out = z_diff;
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

  int n_z = 3;
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  VectorXd z_pred = VectorXd(n_z);
  MatrixXd S = MatrixXd(n_z, n_z);
  PredictRadarMeasurement(n_aug_, Xsig_pred_, weights_, &Zsig, &z_pred, &S);
  VectorXd z_diff;
  UpdateRadarState(n_z, meas_package, n_x_, Zsig, z_pred, Xsig_pred_, weights_,
                   S, &x_, &P_, &z_diff);
  ComputeNIS(z_diff, S, radar_NIS_threshold_, &NIS_radar_,
             &radar_more_NIS_counter_, &radar_counter_, &radar_more_NIS_rate_);
}

void UKF::PredictRadarMeasurement(const int &n_aug_, const MatrixXd &Xsig_pred_,
                                  VectorXd &weights_, MatrixXd *Zsig_out,
                                  VectorXd *z_out, MatrixXd *S_out) {
  ///////////////  Predict Radar Measurement /////////////
  // set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;
  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) { // 2n+1 simga points
    // extract values for better readibility
    double p_x = Xsig_pred_(0, i);
    double p_y = Xsig_pred_(1, i);
    double v = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);
    double v1 = cos(yaw) * v;
    double v2 = sin(yaw) * v;
    // measurement model
    Zsig(0, i) = sqrt(p_x * p_x + p_y * p_y);                         // r
    Zsig(1, i) = atan2(p_y, p_x);                                     // phi
    Zsig(2, i) = (p_x * v1 + p_y * v2) / sqrt(p_x * p_x + p_y * p_y); // r_dot
  }
  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }
  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) { // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // angle normalization
    while (z_diff(1) > M_PI)
      z_diff(1) -= 2. * M_PI;
    while (z_diff(1) < -M_PI)
      z_diff(1) += 2. * M_PI;
    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z, n_z);
  R << std_radr_ * std_radr_, 0, 0, 0, std_radphi_ * std_radphi_, 0, 0, 0,
      std_radrd_ * std_radrd_;
  S = S + R;
  *Zsig_out = Zsig;
  *z_out = z_pred;
  *S_out = S;
}

void UKF::UpdateRadarState(const int &n_z,
                           const MeasurementPackage &meas_package,
                           const int &n_x_, MatrixXd &Zsig,
                           const MatrixXd &z_pred, const MatrixXd &Xsig_pred,
                           const VectorXd &weights_, const MatrixXd &S,
                           VectorXd *x_, MatrixXd *P_, VectorXd *z_diff_out) {
  // create vector for incoming radar measurement
  VectorXd z = VectorXd(n_z);
  z = meas_package.raw_measurements_;
  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  // calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) { // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // angle normalization
    while (z_diff(1) > M_PI)
      z_diff(1) -= 2. * M_PI;
    while (z_diff(1) < -M_PI)
      z_diff(1) += 2. * M_PI;
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - *x_;
    // angle normalization
    while (x_diff(3) > M_PI)
      x_diff(3) -= 2. * M_PI;
    while (x_diff(3) < -M_PI)
      x_diff(3) += 2. * M_PI;
    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K;
  MatrixXd K = Tc * S.inverse();
  // residual
  VectorXd z_diff = z - z_pred;
  // angle normalization
  while (z_diff(1) > M_PI)
    z_diff(1) -= 2. * M_PI;
  while (z_diff(1) < -M_PI)
    z_diff(1) += 2. * M_PI;
  // update state mean and covariance matrix
  *x_ = *x_ + K * z_diff;
  *P_ = *P_ - K * S * K.transpose();
  *z_diff_out = z_diff;
}

void UKF::ComputeNIS(const VectorXd &z_diff, const MatrixXd &S,
                     const double &NIS_threshold_, double *NIS_out,
                     int *more_NIS_counter_, int *counter_,
                     double *more_NIS_rate_) {
  // NIS
  *NIS_out = z_diff.transpose() * S.inverse() * z_diff;
  // Radar NIS threshold check
  if (*NIS_out >= NIS_threshold_) {
    *more_NIS_counter_ = *more_NIS_counter_ + 1;
  }
  // Radar data counter update
  *counter_ = *counter_ + 1;
  // Radar more NIS rate
  *more_NIS_rate_ = double(*more_NIS_counter_) / double(*counter_);
}