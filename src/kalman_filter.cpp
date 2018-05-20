#include "kalman_filter.h"
#define EPS 1e-4

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

/* void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}
*/
void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::UpdateLKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  UpdateKF(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  VectorXd z_pred(3);
  
  //avoid division by zero
  if (fabs(x_(0))<EPS)
	  x_(0)=EPS*copysign(1,x_(0));
  if (fabs(x_(1))<EPS)
	  x_(1)=EPS*copysign(1,x_(1));
  
  z_pred(0) = sqrt(x_(0)*x_(0)+x_(1)*x_(1));
  z_pred(1) = atan2(x_(1),x_(0));
  z_pred(2) = (x_(0)*x_(2)+x_(1)*x_(3))/z_pred(0);
  
  // angles between -PI and PI
  while (z_pred(1)>M_PI)
	z_pred(1)-=2*M_PI;
  while (z_pred(1)<-M_PI)
	z_pred(1)+=2*M_PI;
  
  VectorXd y = z - z_pred;
  UpdateKF(y);

}

void KalmanFilter::UpdateKF(const VectorXd &y) {

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  
  //new estimate
  x_ = x_ + (K * y);
  P_ = (I_ - K * H_) * P_;
 
 }