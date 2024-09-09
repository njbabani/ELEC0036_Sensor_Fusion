// LowPass.h

#ifndef LOWPASS_H
#define LOWPASS_H

#include <Arduino.h>

template<int order>  // order is 1 or 2 or 4
class LowPass {
private:
  float a[order];
  float b[order + 1];
  float omega0;
  float dt;
  bool adapt;
  float tn1 = 0;
  float x[order + 1];  // Raw values
  float y[order + 1];  // Filtered values

public:
  LowPass(float f0, float fs, bool adaptive) {
    // f0: cutoff frequency (Hz)
    // fs: sample frequency (Hz)
    // adaptive: boolean flag, if set to 1, the code will automatically set
    // the sample frequency based on the time history.

    omega0 = 6.28318530718 * f0;
    dt = 1.0 / fs;
    adapt = adaptive;
    tn1 = -dt;
    for (int k = 0; k < order + 1; k++) {
      x[k] = 0;
      y[k] = 0;
    }
    setCoef();
  }

  void setCoef() {
    if (adapt) {
      float t = micros() / 1.0e6;
      dt = t - tn1;
      tn1 = t;
    }

    float alpha = omega0 * dt;
    if (order == 1) {
      a[0] = -(alpha - 2.0) / (alpha + 2.0);
      b[0] = alpha / (alpha + 2.0);
      b[1] = alpha / (alpha + 2.0);
    }
    if (order == 2) {
      float alphaSq = alpha * alpha;
      float beta[] = { 1, sqrt(2), 1 };
      float D = alphaSq * beta[0] + 2 * alpha * beta[1] + 4 * beta[2];
      b[0] = alphaSq / D;
      b[1] = 2 * b[0];
      b[2] = b[0];
      a[0] = -(2 * alphaSq * beta[0] - 8 * beta[2]) / D;
      a[1] = -(beta[0] * alphaSq - 2 * beta[1] * alpha + 4 * beta[2]) / D;
    }
    if (order == 4) {
      float beta[] = { 1, 2.61312593, 3.414213562, 2.61312593, 1 };
      float alpha_2 = alpha * alpha;
      float alpha_3 = alpha * alpha * alpha;
      float alpha_4 = alpha * alpha * alpha * alpha;
      float D = 4 * alpha_4 * beta[0] + 2 * alpha_3 * beta[1] + 4 * alpha_2 * beta[2] + 8 * alpha * beta[3] + 16 * beta[4];
      b[0] = alpha_4 / D;
      b[1] = 4 * b[0];
      b[2] = 6 * b[0];
      b[3] = 4 * b[0];
      b[4] = b[0];
      a[0] = (-4 * alpha_4 * beta[0] - 4 * alpha_3 * beta[1] + 16 * alpha * beta[3] + 64 * beta[4]) / D;
      a[1] = (-6 * alpha_4 * beta[0] + 8 * alpha_2 * beta[2] - 96 * beta[4]) / D;
      a[2] = (-4 * alpha_4 * beta[0] + 4 * alpha_3 * beta[1] - 16 * alpha * beta[3] + 64 * beta[4]) / D;
      a[3] = (-alpha_4 * beta[0] + 2 * alpha_3 * beta[1] - 4 * alpha_2 * beta[2] + 8 * alpha * beta[3] - 16 * beta[4]) / D;
    }
  }

  float filt(float xn) {
    // Provide me with the current raw value: x
    // I will give you the current filtered value: y
    if (adapt) {
      setCoef();  // Update coefficients if necessary
    }
    y[0] = 0;
    x[0] = xn;
    // Compute the filtered values
    for (int k = 0; k < order; k++) {
      y[0] += a[k] * y[k + 1] + b[k] * x[k];
    }
    y[0] += b[order] * x[order];

    // Save the historical values
    for (int k = order; k > 0; k--) {
      y[k] = y[k - 1];
      x[k] = x[k - 1];
    }

    // Return the filtered value
    return y[0];
  }
};

#endif
