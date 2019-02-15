#include <iostream>
#include <limits>
#include "pid.h"

// Constructor
PID::PID() {
  this->Kp = DEFAULT_KP;
  this->Ki = DEFAULT_KI;
  this->Kd = DEFAULT_KD;
  this->int_val = 0;
  this->last_err = 0;
  this->min_val = std::numeric_limits<int>::min();;
  this->max_val = std::numeric_limits<int>::max();;
}

// Init
void PID::init(float Kp, float Ki, float Kd, float min_val, float max_val) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  this->min_val = min_val;
  this->max_val = max_val;
}

// Reset func
void PID::reset() {
  // Reset the integral term so that the controller does not accumulate errors
  this->int_val = 0;

}

// Apply the controller
float PID::step(float err, float sample_time) {
  // compute integral
  float integral = this->int_val + (err * sample_time);

  // compute derivative
  float derivative = (err - this->last_err) / sample_time;

  // output of PID controller
  float val = (this->Kp * err) + (this->Ki * integral) + (this->Kd * derivative);
  // check if the value to be returned is in range
  if (val > this->max_val)
      val = this->max_val;
  else if (val < this->min_val)
      val = this->min_val;
  else
      this->int_val = integral;

  // update last error
  this->last_err = err;

  return val;
}
