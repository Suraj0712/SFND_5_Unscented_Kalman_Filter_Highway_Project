#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() 
{
  
  use_laser_ = true;      // if this is false, laser measurements will be ignored (except during init)
  use_radar_ = true;      // if this is false, radar measurements will be ignored (except during init)
  x_ = VectorXd(5);       // initial state vector
  P_ = MatrixXd(5, 5);    // initial covariance matrix
  std_a_ = 30;            // Process noise standard deviation longitudinal acceleration in m/s^2
  std_yawdd_ = 30;        // Process noise standard deviation yaw acceleration in rad/s^2
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */
  std_laspx_ = 0.15;      // Laser measurement noise standard deviation position1 in m
  std_laspy_ = 0.15;     // Laser measurement noise standard deviation position2 in m
  std_radr_ = 0.3;       // Radar measurement noise standard deviation radius in m
  std_radphi_ = 0.03;   // Radar measurement noise standard deviation angle in rad
  std_radrd_ = 0.3;     // Radar measurement noise standard deviation radius change in m/s
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */

  x_.fill(0.0);

  P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;

  std_a_ = 2;
  std_yawdd_ = 0.7;
  
  is_initialized_ = false;

  n_x_ = 5;                                     // State dimension
  n_aug_ = 7;                                   // Augmented state dimension
  lambda_ = 3 - n_x_;                           // Sigma point spreading parameter
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);  // predicted sigma points matrix
  Xsig_pred_.fill(0.0); 
  time_us_ = 0.0;                               // last step time in us

  // Weights of sigma points
  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_.fill(0.5 / (lambda_ + n_aug_));
  weights_(0) = lambda_ / (lambda_ + n_aug_);
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) 
{
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */

  if (is_initialized_ == false)
  {
    if (meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
      x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;    // initialize using lidar
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
      double rho = meas_package.raw_measurements_[0];
      double phi = meas_package.raw_measurements_[1];
      double rho_dot = meas_package.raw_measurements_[2];

      x_ << rho * cos(phi), rho * sin(phi), 0, 0, 0;                                           // initialize using radar
    }
    else
    {
      std::cout << "Invalid measurement \n";
    }

    is_initialized_ = true;
    time_us_ = meas_package.timestamp_;

    return;
  }

  double dt = static_cast<double>((meas_package.timestamp_ - time_us_) * 1e-6);
  time_us_ = meas_package.timestamp_;

  // prediction
  Prediction(dt);

  // update
  if (meas_package.sensor_type_ == MeasurementPackage::LASER)
  { 
    UpdateLidar(meas_package);
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
  {
    UpdateRadar(meas_package);
  }
  else
  {
    std::cout << "Invalid measurement \n";
  }
}


void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
}