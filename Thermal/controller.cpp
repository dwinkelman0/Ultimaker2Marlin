#include "controller.h"

#include "Thermal.h"
#include "bioprinter_thermistors.h"

static float convertAnalogToTemp(const uint16_t analog) {
  uint16_t offset = analog & 0x1fff;
  if (offset < 0x0400) {
    offset = 0x0000;
  }
  else if (offset >= 0x1c00) {
    offset = 0x17ff;
  }
  else {
    offset -= 0x0400;
  }
  return pgm_read_float(bioprinter_thermistor_table + offset);
}

Controller::Controller() : Controller(0, 0, 1.0f, 1.0f, 1.0f) {}

Controller::Controller(const int inputPin, const int outputPin, const float p, const float i, const float d) :
    inputPin_(inputPin), outputPin_(outputPin),
    p_(p), i_(i), d_(d),
    currentTemp_(0),
    targetTemp_(0),
    integralTemp_(0),
    index_(0),
    maxTemp_(999) {
  // Important to not use a pull-up resistor
  if (inputPin >= 0) {
    pinMode(inputPin, INPUT);
  }
  if (outputPin >= 0) {
    pinMode(outputPin, OUTPUT);
  }
  memset(errors_, sizeof(errors_), 0);
  memset(times_, sizeof(times_), 0);
}

void Controller::adjustPower() {
  // Time is always measured from the start of an analog conversion
  if (inputPin_ < 0) return;
  uint32_t currentTime = micros();

  // Oversample to get better resolution (13 bits)
  int currentTempRaw = 0;
  for (int i = 0; i < 8; ++i) {
    currentTempRaw += analogRead(inputPin_);
  }
  currentTemp_ = convertAnalogToTemp(currentTempRaw);
  errors_[index_] = currentTemp_ - targetTemp_;
  times_[index_] = currentTime * 1e-6;
  index_ = (index_ + 1) % NUM_HISTORY;

  float derivative = calcDerivative();
  
  //Serial.println(String("temp: ") + String(currentTemp_));
  Serial.println(String("time: ") + String(derivative));

  // Calculate PID parameters
  if (outputPin_ < 0) return;
}

float Controller::calcDerivative() const {
  // Calculate means; it is OK to compute over all elements without knowing
  // validity because they are all initialized to zero
  //
  // time -> X
  // error -> Y
  float meanError = 0.0f;
  float meanTime = 0.0f;
  for (int i = 0; i < NUM_HISTORY; ++i) {
    meanError += errors_[i];
    meanTime += times_[i];
  }
  meanError /= NUM_HISTORY;
  meanTime /= NUM_HISTORY;

  // Calculate slope
  float dividend = 0.0f;
  float divisor = 0.0f;
  for (int i = 0; i < NUM_HISTORY; ++i) {
    float diffTime = times_[i] - meanTime;
    dividend += diffTime * (errors_[i] - meanError);
    divisor += diffTime * diffTime;
  }
  return dividend / divisor;
}
