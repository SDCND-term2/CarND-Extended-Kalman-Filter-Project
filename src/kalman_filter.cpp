#include <iostream>
#include "kalman_filter.h"

using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  TODO:
    * predict the state
  */

  x_ = F_ * x_; // section 8 in lesson 5
  MatrixXd Ft = F_.transpose(); // section 9 in lesson 5
  P_ = F_ * P_ * Ft + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */

  // I section 7 of lesson 5
  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;


  // new state
  x_ = x_ + (K * y);
  MatrixXd I = MatrixXd(4,4);
  I << 1, 0, 0, 0,
       0, 1, 0, 0,
       0, 0, 1, 0,
       0, 0, 0, 1;

  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  // Section 14 of lesson 5
  //
  float x = x_(0);
  float y = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  float rho = sqrt(x*x+y*y);
  float theta = atan2(y,x);
  float ro_dot = (x*vx+y*vy)/rho;
  VectorXd z_pred = VectorXd(3);
  z_pred << rho, theta,ro_dot;

  VectorXd Y = z - z_pred;

  // angle normalization
  while (Y(1)> M_PI) Y(1)-=2.*M_PI;
  while (Y(1)<-M_PI) Y(1)+=2.*M_PI;

  // In section 7 of lesson 5
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;

  // new state
  MatrixXd I = MatrixXd(4,4);
  I << 1, 0, 0, 0,
       0, 1, 0, 0,
       0, 0, 1, 0,
       0, 0, 0, 1;

  x_ = x_ + (K * Y);

  P_ = (I - K * H_) * P_;
}
