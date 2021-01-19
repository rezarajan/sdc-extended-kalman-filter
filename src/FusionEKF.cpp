#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF()
{
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
      0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
      0, 0.0009, 0,
      0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
  //laser measurement matrix
  H_laser_ << 1, 0, 0, 0,
      0, 1, 0, 0;

  // radar Jacobian
  // Hj_ = tools.CalculateJacobian();

  // set the acceleration noise components
  noise_ax = 9;
  noise_ay = 9;

  // Default state initialization
  ekf_.x_ = VectorXd(4);
  ekf_.x_ << 0, 0, 0, 0;

}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack)
{
  /**
   * Initialization
   */
  if (!is_initialized_)
  {
    /**
     * Initialize the state ekf_.x_ with the first measurement.
     * Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);

    // state covariance matrix P
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
               0, 1, 0, 0,
               0, 0, 1, 0,
               0, 0, 0, 1;

    ekf_.Q_ = MatrixXd(4, 4);
    

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
      // TODO: Convert radar from polar to cartesian coordinates
      //         and initialize state.
      // px, py, vx, vy
      cout<<"Initializing RADAR\n";
      ekf_.x_(0) = measurement_pack.raw_measurements_[0]*cos(measurement_pack.raw_measurements_[1]);
      ekf_.x_(1) = measurement_pack.raw_measurements_[0]*sin(measurement_pack.raw_measurements_[1]);
      ekf_.x_(2) = 0;
      ekf_.x_(3) = 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
    {
      // TODO: Initialize state.
      // set the state with the initial location and zero velocity
      cout<<"Initializing LASER\n";
      ekf_.x_ << measurement_pack.raw_measurements_[0],
          measurement_pack.raw_measurements_[1],
          0,
          0;
    }

    previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  // Elapsed between measurements, in seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  // State transition function updated with current delta t
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, dt, 0,
             0, 1, 0, dt,
             0, 0, 1, 0,
             0, 0, 0, 1;

  // Current Process Noise Covariance Matrix updated with delta t
  ekf_.Q_ << (pow(dt, 4) * noise_ax / 4), 0, (pow(dt, 3) * noise_ax / 2), 0,
      0, (pow(dt, 4) * noise_ay / 4), 0, (pow(dt, 3) * noise_ay / 2),
      (pow(dt, 3) * noise_ax / 2), 0, (pow(dt, 2) * noise_ax), 0,
      0, (pow(dt, 3) * noise_ay / 2), 0, (pow(dt, 2) * noise_ay);
  
  ekf_.Predict();

  /**
   * Update
   */

  /**
   * 
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
  {

    // Calculating Jacobian
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    // TODO: Radar updates
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  }
  else
  {
    // TODO: Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
