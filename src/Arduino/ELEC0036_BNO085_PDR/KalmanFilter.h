#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <Arduino.h>
#include <math.h>
#include <LinearAlgebra.h>

class KalmanFilter {
private:
  mat x;  // State vector [x, y, λ, ψ]
  mat P;  // Error covariance matrix
  mat F;  // State transition matrix
  mat Q;  // Process noise covariance matrix
  mat H;  // Observation matrix
  mat R;  // Measurement noise covariance matrix
  mat B;  // Control input matrix
  mat u;  // Control vector
  mat I;  // Identity matrix
  mat S;  // Innovation covariance matrix
  mat K;  // Kalman gain matrix
  mat y;  // Residual matrix
  mat z;  // Measurement vector in matrix form

public:
  KalmanFilter(int state_size, int measurement_size) {
    x = mat::zeros(state_size, 1);                       // Initialise with zeros or initial state estimate
    P = mat::identity(state_size);                       // Initialise with some uncertainty
    F = mat::identity(state_size);                       // Initialised as identity
    Q = mat::identity(state_size);                       // Define how noisy your process is
    H = mat::zeros(measurement_size, state_size);        // Will be filled with appropriate values
    R = mat::identity(measurement_size);                 // Define the measurement noise
    B = mat::zeros(state_size, state_size);              // If no control input, this remains zero
    u = mat::zeros(state_size, 1);                       // If no control input, this remains zero
    I = mat::identity(state_size);                       // Identity matrix for updates
    S = mat::zeros(measurement_size, measurement_size);  // Initialise with zeros
    K = mat::zeros(measurement_size, measurement_size);  // Initialise with zeros
    z = mat::zeros(measurement_size, 1);                 // Initialise measurement matrix with zeros
  }

  // Method to define initial state values
  void initialStateEstimate(mat &x0) {
    x = x0;
  }

  // Method defining initial error covariance matrix
  void initialErrorCovariance(mat &P0) {
    P = P0;
  }

  // Method to define the state transition matrix
  void setStateTransition(const mat &F_) {
    F = F_;
  }

  // Method to define the state space model
  void setStateSpace(const mat &x_) {
    x = x_;
  }

  // Method to set the control input matrix B and control vector u
  void setControlInput(const mat &B_, const mat &u_) {
    B = B_;
    u = u_;
  }

  // Method to set the process and measurement noise matrices
  void setNoiseCovariances(const double omega, const double sigma) {
    Q = Q * omega;  // Multiply by process noise omega
    R = R * sigma;  // Multiply by measurement noise sigma
  }

  // Method to set the observation matrix H
  void setObservationMatrix(const mat &H_) {
    H = H_;
  }

  // The prediction step of the Kalman filter
  void predict() {
    // Predict the state and error covariance
    x = F * x + B * u;      // Predicted (a priori) state estimate
    P = F * P * F.t() + Q;  // Predicted (a priori) error covariance matrix
  }

  // The update step of the Kalman filter
  void update(const mat &z_) {
    // z is the measurement vector
    z = z_;

    // Compute the Kalman Gain
    S = H * P * H.t() + R;    // Innovation covariance
    K = P * H.t() * S.inv();  // Kalman gain

    // Update the estimate via measurement from z
    y = z - H * x;        // Measurement residual
    x = x + K * y;        // Updated (a posteriori) state estimate
    P = (I - K * H) * P;  // Updated (a posteriori) error covariance matrix
  }
};

#endif