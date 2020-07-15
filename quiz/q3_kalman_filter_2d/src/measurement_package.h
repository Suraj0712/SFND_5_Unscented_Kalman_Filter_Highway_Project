#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "/home/sur/SFND_5_Unscented_Kalman_Filter_Highway_Project/Eigen/Dense"

class MeasurementPackage {
 public:

  enum SensorType {
    LASER, RADAR
  } sensor_type_;

  Eigen::VectorXd raw_measurements_;
  
  int64_t timestamp_;

};

#endif  // MEASUREMENT_PACKAGE_H_
