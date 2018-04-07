#ifndef UKF_H
#define UKF_H

#include "Eigen/Dense"
#include "measurement_package.h"
#include <fstream>
#include <string>
#include <vector>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:
  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* time when the state is true, in us
  long long time_us_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Sigma point spreading parameter
  double lambda_;

  ///* NIS Radar
  double NIS_radar_;

  ///* NIS Lasar
  double NIS_laser_;

  ///* Radar data counter
  int radar_counter_;

  ///* Radar NIS threshold
  double radar_NIS_threshold_;

  ///* Radar more NIS counter
  int radar_more_NIS_counter_;

  ///* Radar more NIS rate
  double radar_more_NIS_rate_;

  ///* Lasar data counter
  int laser_counter_;

  ///* Lasar NIS threshold
  double laser_NIS_threshold_;

  ///* Lasar more NIS counter
  int laser_more_NIS_counter_;

  ///* Lasar more NIS rate
  double laser_more_NIS_rate_;

  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  void GenerateSigmaPoints(const int &n_x_, const MatrixXd &P_,
                           const double &lambda_, const VectorXd &x_,
                           MatrixXd *Xsig_out);
  void AugmentedSigmaPoints(const int &n_aug_, const VectorXd &x_,
                            const MatrixXd &P_, const double &std_a_,
                            const double &std_yawdd_, const double &lambda_,
                            MatrixXd *Xsig_aug_out);
  void SigmaPointPrediction(const int &n_aug_, const MatrixXd &Xsig_aug,
                            const double &delta_t, MatrixXd *Xsig_pred_out);
  void PredictMeanAndCovariance(VectorXd &weights_, const int &n_aug_,
                                const double &lambda_, MatrixXd &Xsig_pred_,
                                VectorXd *x_out, MatrixXd *P_out);
  /**
   * Updates the state and the state covariance matrix using a laser
   * measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);

  void PredictLidarMeasurement(const int &n_aug_, const MatrixXd &Xsig_pred_,
                               VectorXd &weights_, MatrixXd *Zsig_out,
                               VectorXd *z_out, MatrixXd *S_out);
  void UpdateLidarState(const int &n_z, const MeasurementPackage &meas_package,
                        const int &n_x_, MatrixXd &Zsig, const MatrixXd &z_pred,
                        const MatrixXd &Xsig_pred, const VectorXd &weights_,
                        const MatrixXd &S, VectorXd *x_, MatrixXd *P_,
                        VectorXd *z_diff_out);

  /**
   * Updates the state and the state covariance matrix using a radar
   * measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);

  void PredictRadarMeasurement(
      const int &n_aug_, const MatrixXd &Xsig_pred_, VectorXd &weights_,
      MatrixXd *Zsig_out, VectorXd *z_out,
      MatrixXd *S_out); // create matrix for sigma points in measurement space

  void UpdateRadarState(const int &n_z, const MeasurementPackage &meas_package,
                        const int &n_x_, MatrixXd &Zsig, const MatrixXd &z_pred,
                        const MatrixXd &Xsig_pred, const VectorXd &weights_,
                        const MatrixXd &S, VectorXd *x_, MatrixXd *P_,
                        VectorXd *z_diff_out);
  void ComputeNIS(const VectorXd &z_diff, const MatrixXd &S,
                  const double &NIS_threshold_, double *NIS_out,
                  int *more_NIS_counter_, int *counter_,
                  double *more_NIS_rate_);
};

#endif /* UKF_H */
