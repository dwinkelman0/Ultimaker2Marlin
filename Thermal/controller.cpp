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

Controller::Controller() : Controller(NULL, 0) {}

Controller::Controller(const char *code, const int inputPin) : Controller(code, inputPin, 0, 0.0f, 0.0f, 0.0f) {}

Controller::Controller(const char *code, const int inputPin, const int outputPin, const float p, const float i, const float d) :
    inputPin_(inputPin), outputPin_(outputPin),
    p_(p), i_(i), d_(d),
    currentTemp_(0),
    targetTemp_(20),
    integralTemp_(0),
    index_(0),
    maxTemp_(999) {

  // Set identification code
  if (code) {
    code_[0] = code[0] ? code[0] : '.';
    code_[1] = code[1] && code[0] ? code[1] : '.';
    code_[2] = code[2] && code[1] && code[0] ? code[2] : '.';
  }
  else {
    code_[0] = code_[1] = code_[2] = '.';
  }
  code_[3] = '\0';
  
  // Important to not use a pull-up resistor
  if (inputPin >= 0) {
    pinMode(inputPin, INPUT);
  }
  if (outputPin >= 0) {
    pinMode(outputPin, OUTPUT);
  }
  reset();
}

void Controller::printTemp() const {
  Serial.print(code_);
  Serial.print(" ");
  Serial.println(String(currentTemp_));
}

void Controller::adjustPower() {
  // Time is always measured from the start of an analog conversion
  if (inputPin_ < 0) return;
  float currentTime = micros() * 1e-6;

  // Oversample to get better resolution (13 bits)
  int currentTempRaw = 0;
  for (int i = 0; i < 8; ++i) {
    currentTempRaw += analogRead(inputPin_);
  }
  currentTemp_ = convertAnalogToTemp(currentTempRaw);

  // Exit now if there is no output pin (i.e. this is only a sensor)
  if (outputPin_ < 0) return;

  // Compute errors and update state
  float error = currentTemp_ - targetTemp_;
  float prevTime = times_[index_ == 0 ? NUM_HISTORY - 1 : index_ - 1];
  errors_[index_] = error;
  times_[index_] = currentTime;
  index_ = (index_ + 1) % NUM_HISTORY;

  // Check for maximum temperature
  if (currentTemp_ > maxTemp_) {
    analogWrite(outputPin_, 0);
    return;
  }

  // Calculate PID parameters
  float derivative = calcDerivative();
  integralTemp_ += error * (currentTime - prevTime) / 2;
  float pTerm = p_ * error;
  float iTerm = constrain(i_ * integralTemp_, -128, 0);
  float dTerm = constrain(d_ * derivative, -255, 255);
  int16_t power = -(pTerm + iTerm + dTerm);
  analogWrite(outputPin_, constrain(power, 0, 255));

  #if 0
  // Debug contributions of factors to power
  Serial.print(String(error));
  Serial.print(" ");
  Serial.print(String(-pTerm / 25.5f));
  Serial.print(" ");
  Serial.print(String(-iTerm / 25.5f));
  Serial.print(" ");
  Serial.print(String(-dTerm / 25.5f));
  Serial.println("");
  #endif
}

void Controller::reset() {
  integralTemp_ = 0.0f;
  index_ = 0;
  memset(errors_, sizeof(errors_), 0);
  memset(times_, sizeof(times_), 0);
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
