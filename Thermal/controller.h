#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include "Thermal.h"

/* Controller for a closed temperature loop; can either measure and
 *  adjust or only measure; all numbers use units of degrees Celsius
 *  and seconds
 */
class Controller {
 public:
  Controller();
  Controller(const int inputPin, const int outputPin, const float p, const float i, const float d);

  /* Set the target temperature */
  inline void setTargetTemp(const float temp) {
    targetTemp_ = temp;
  }

  /* Set the temperature at which heating stops */
  inline void setMaxTemp(const float temp) {
    maxTemp_ = temp;
  }

  /* Get most recent temperature measurement.
     Note: does not perform a new measurement, returns previousTemp_ */
  inline float getTemp() const {
    return currentTemp_;
  }

  /* Get current temperature and adjust power pulse width modulation */
  void adjustPower();

 private:
  /* Use least-squares to calculate the recent slope of error;
   *  https://www.varsitytutors.com/hotmath/hotmath_help/topics/line-of-best-fit
   */
  float calcDerivative() const;

 private:
  /* Pins */
  int inputPin_;
  int outputPin_;
  
  /* PID parameters */
  float p_, i_, d_;

  /* PID state; store history of recent measurements to calculate smoother
   *  derivatives */
  static const int NUM_HISTORY = 16;
  float currentTemp_;
  float errors_[NUM_HISTORY];
  float times_[NUM_HISTORY];
  int index_;
  float targetTemp_;
  float integralTemp_;

  /* Miscellaneous */
  float maxTemp_;
};

#endif
