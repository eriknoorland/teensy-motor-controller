#include "Arduino.h"
#include "MotorController.h"
#include "PID_v1.h"

/**
 * MotorController
 * @param maxSpeed
 * @param loopTime
 */
MotorController::MotorController() : _speedPID(&_speedTicksInput, &_speedPwmOutput, &_speedSetpoint, 3, 90, 0.1, DIRECT) {
  _speedPID.SetMode(AUTOMATIC);
  _numTicks = 0;
  _numLastTicks = 0;
  _direction = 1;
  _lastError = 0;
  _accumulatedError = 0;
}

/**
 * Setup
 * @param enable
 * @param enableB
 * @param pwm1
 * @param pwm2
 * @param encoderA
 * @param encoderB
 */
void MotorController::setup(int enable, int enableB, int pwm1, int pwm2, int encoderA, int encoderB) {
  _pwm1Pin = pwm1;
  _pwm2Pin = pwm2;

  pinMode(enable, OUTPUT);
  pinMode(enableB, OUTPUT);
  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);

  digitalWrite(enable, HIGH);
  digitalWrite(enableB, LOW);
}

/**
 * Set the pid loop samle time
 * @param loopTime
 */
void MotorController::setLoopTime(int loopTime) {
  _speedPID.SetSampleTime(loopTime);
}

/**
 * Set the pid loop samle time
 * @param loopTime
 */
void MotorController::setControlLimits(int min, int max) {
  _speedPID.SetOutputLimits(min, max);
  _pwmMinLimit = min;
  _pwmMaxLimit = max;
}

/**
 * Loop
 */
void MotorController::loop() {
  bool pidEnabled = _speedPID.GetMode() == AUTOMATIC;

  if (pidEnabled) {
    if (_speedPID.Compute()) {
      motorControl(round(_speedPwmOutput), _direction);
    }

    return;
  }

  motorControl(0, 0);
}

/**
 * Motor control
 * @param {int} pwm
 * @param {int} direction
 */
void MotorController::motorControl(int pwm, int direction) {
  analogWrite(_pwm1Pin, direction == 1 ? pwm : 0);
  analogWrite(_pwm2Pin, direction == -1 ? pwm : 0);
}

/**
 * Set the motor direction
 * @param {int} direction
 */
void MotorController::setDirection(int direction) {
  _direction = direction == 1 ? direction : -1;
}

/**
 * Returns the motor direction
 * @return int
 */
int MotorController::getDirection() {
  return _direction == 1 ? _direction : 0;
}

/**
 * Set the motor speed
 * @param {int} speed
 */
void MotorController::setSpeed(int speed) {
  _speedPID.SetMode(AUTOMATIC);
  _speedSetpoint = speed;
}

/**
 * Set the motor pwm
 * @param {int} speed
 * @param {int} direction
 */
void MotorController::setPwm(int pwm, int direction) {
  int contrainedPwm = constrain(pwm, _pwmMinLimit, _pwmMaxLimit);
  _speedPID.SetMode(MANUAL);

  motorControl(contrainedPwm, direction == 1 ? direction : -1);
}

/**
 * Hard stop the motor
 */
void MotorController::stop() {
  _speedPID.SetMode(MANUAL);
  _speedSetpoint = 0;
  _speedPwmOutput = 0;
}

/**
 * Controls the speed and returns the number of ticks since the last time requested
 * @return int
 */
int MotorController::update() {
  int deltaTicks = _numTicks - _numLastTicks;

  _speedTicksInput= deltaTicks;
  _numLastTicks = _numTicks;

  return deltaTicks;
}

/**
 * Encoder tick event handler
 */
void MotorController::onEncoderTick() {
  _numTicks += 1;
}
