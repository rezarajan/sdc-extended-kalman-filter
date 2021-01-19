#include "kalman_filter.h"
#include <iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  x_ = F_*x_;
  P_ = F_*P_*F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  // recover state parameters
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  // state space to measurement space mapping
  float rho = sqrt(px*px + py*py);
  float phi = atan2(py,px);
  float rho_dot = (px*vx + py*vy)/rho;
  
  std::cout<<"State Phi: "<<phi<<"\n";
  
  VectorXd z_mod = z;
  // fixing the angle range
  if(z_mod(1) > M_PI){
    // adjust to range(-pi,0)
    z_mod(1) -= 2*M_PI;
    if(phi > 0){
      // case where phi is in range(0,pi)
      // but z is not
      // change phi to result in same error value
      phi -= 2*M_PI;
    }
  }
  else if(z_mod(1) < -M_PI){
    // adjust to range(0,pi)
    z_mod(1) += 2*M_PI;
    if(phi < 0){
      // case where phi is in range(-pi,0)
      // but z is not
      // change phi to result in same error value
      phi += 2*M_PI;
    }
  }
  std::cout<<"Phi: "<<z(1)<<"\n";

  VectorXd h(3);
  h << rho, phi, rho_dot;

  VectorXd z_pred = h;
  VectorXd y = z_mod - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  
  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
