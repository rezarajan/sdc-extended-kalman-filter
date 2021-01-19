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
  // std::cout<<"x_:\n"<<x_<<"\n\n";
  // std::cout<<"F_:\n"<<F_<<"\n\n";
  std::cout<<"Predicting State\n";
  x_ = F_*x_;
  P_ = F_*P_*F_.transpose() + Q_;
  // std::cout<<"x_:\n"<<x_<<"\n\n";
  // std::cout<<"P_:\n"<<P_<<"\n\n";
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  std::cout<<"KF Update\n";
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  // std::cout<<"y\n";
  MatrixXd Ht = H_.transpose();
  // std::cout<<"Ht\n"<<Ht<<"\n";
  // std::cout<<"Debugging R\n"<<R_<<"\n";
  // std::cout<<"Debugging P_\n"<<H_*P_*Ht<<"\n";
  MatrixXd S = H_ * P_ * Ht + R_;
  // std::cout<<"S\n";
  MatrixXd Si = S.inverse();
  // std::cout<<"Si\n";
  MatrixXd PHt = P_ * Ht;
  // std::cout<<"PHt\n";
  MatrixXd K = PHt * Si;
  // std::cout<<"K\n";

  //new estimate
  x_ = x_ + (K * y);
  // std::cout<<"x_\n";
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  // std::cout<<"P_\n";
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  std::cout<<"EKF Update\n";
  // recover state parameters
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  std::cout<<"Calculating Rho's\n";
  // state space to measurement space mapping
  float rho = sqrt(px*px + py*py);
  float phi = atan2(py,px);
  float rho_dot = (px*vx + py*vy)/rho;
  
  std::cout<<"Calculating Kalman Gain\n";

  VectorXd h;
  h << rho, phi, rho_dot;
  std::cout<<"h mapping created\n";

  VectorXd z_pred = h * x_;
  std::cout<<"z_pred\n";
  VectorXd y = z - z_pred;
  std::cout<<"y\n";
  MatrixXd Ht = H_.transpose();
  std::cout<<"Ht\n";
  MatrixXd S = H_ * P_ * Ht + R_;
  std::cout<<"S\n";
  MatrixXd Si = S.inverse();
  std::cout<<"Si\n";
  MatrixXd PHt = P_ * Ht;
  std::cout<<"PHt\n";
  MatrixXd K = PHt * Si;
  std::cout<<"K\n";
  
  std::cout<<"Updating State\n";
  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  std::cout<<"State Successfully Updated\n";
}
