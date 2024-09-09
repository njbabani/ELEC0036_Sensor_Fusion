#ifndef EXTENDEDKALMANFILTER_H
#define EXTENDEDKALMANFILTER_H

#include <Arduino.h>
#include <math.h>
#include <LinearAlgebra.h>
#include "UserPosition.h"

class ExtendedKalmanFilter {
private:
  mat F;       // State transiton matrix that maps state vector to state space (Used for calling non-linear process function - F is NOT jacobian)
  mat x;       // State vector [x, y, λ, ψ]
  mat P;       // Error covariance matrix
  mat grad_f;  // Jacobian of process model
  mat Q;       // Process noise covariance matrix
  mat grad_h;  // Jacobian of measurement model
  mat H;       // H matrix maps state space into measurement space (Used for calling non-linear measurement function - H is NOT jacobian)
  mat R;       // Measurement noise covariance matrix
  mat B;       // Control input matrix
  mat u;       // Control vector
  mat I;       // Identity matrix
  mat S;       // Innovation covariance matrix
  mat K;       // Kalman gain matrix
  mat y;       // Residual matrix
  mat z;       // Measurement vector in matrix form

public:
  ExtendedKalmanFilter(int state_size, int measurement_size) {
    F = mat::identity(state_size);                       // Initialise as indentity matrix
    H = mat::zeros(measurement_size, state_size);        // Initialise as indentity matrix
    x = mat::zeros(state_size, 1);                       // Initialise with zeros or initial state estimate
    P = mat::identity(state_size);                       // Initialise with some uncertainty
    grad_f = mat::identity(state_size);                  // Initialised as identity
    Q = mat::identity(state_size);                       // Define how noisy your process is
    grad_h = mat::zeros(measurement_size, state_size);   // Will be filled with appropriate values
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
  void setProcessJacboian(const mat &Jacobi_f) {
    grad_f = Jacobi_f;
  }

  void setStateTransitionMatrix(const mat &F_) {
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
  void setMeasurementJacobian(const mat &Jacobi_h) {
    grad_h = Jacobi_h;
  }

  void setObservationMatrix(const mat &H_) {
    H = H_;
  }

  // The prediction step of the Kalman filter
  void
  predict() {
    // Predict the state and error covariance
    x = F * x + B * u;                // Predicted (a priori) state estimate
    P = grad_f * P * grad_f.t() + Q;  // Predicted (a priori) error covariance matrix
  }

  // The update step of the Kalman filter
  void update(const mat &z_) {
    // z is the measurement vector
    z = z_;

    // Compute the Kalman Gain
    S = grad_h * P * grad_h.t() + R;  // Innovation covariance
    K = P * grad_h.t() * S.inv();     // Kalman gain

    // Update the estimate via measurement from z
    y = z - H * x;        // Measurement residual (you use H to call the measurement function)
    x = x + K * y;        // Updated (a posteriori) state estimate
    P = (I - K * H) * P;  // Updated (a posteriori) error covariance matrix
  }
};

#endif