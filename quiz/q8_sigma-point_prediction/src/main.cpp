#include "/home/sur/SFND_5_Unscented_Kalman_Filter_Highway_Project/Eigen/Dense"
#include "ukf.h"

using Eigen::MatrixXd;

int main() {

  // Create a UKF instance
  UKF ukf;

  /**
   * Programming assignment calls
   */
  MatrixXd Xsig_pred = MatrixXd(15, 5);
  ukf.SigmaPointPrediction(&Xsig_pred);

  return 0;
}
