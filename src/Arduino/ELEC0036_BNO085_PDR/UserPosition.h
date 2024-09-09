#ifndef USERPOSITION_H
#define USERPOSITION_H

#include <Arduino.h>

class UserPosition {
private:
  float x, y;          // Current position
  float lambda;        // Step length
  float psi;           // Heading in degrees

public:
  // Constructor initializes position to (0,0) and step length and heading to 0
  UserPosition()
    : x(0.0), y(0.0), lambda(0.0), psi(0.0) {}

  // Set x position
  void setX(float x_) {
    x = x_;
  }

  // Set y position
  void setY(float y_) {
    y = y_;
  }

  // Set steplength
  void setStepLength(float lambda_) {
    lambda = lambda_;
  }

  // Set heading
  void setHeading(float psi_) {
    psi = psi_;
  }

  // Method to update position based on current step length and heading
  void updatePosition(float step_length, float heading_degrees) {
    // Update steplength and heading
    setStepLength(step_length);
    setHeading(heading_degrees);

    // Update x and y using the provided formula
    x += step_length * cos(heading_degrees * DEG_TO_RAD);
    y += step_length * sin(heading_degrees * DEG_TO_RAD);
  }

  // Getter for x position
  float getX(void) const {
    return x;
  }

  // Getter for y position
  float getY(void) const {
    return y;
  }

  // Getter for steplength
  float getStepLength(void) {
    return lambda;
  }

  // Getter for heading
  float getHeading(void) {
    return psi;
  }
};

#endif