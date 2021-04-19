#ifndef MotorController_h
#define MotorController_h

#include "Arduino.h"
#include "PID_v1.h"

struct MotorDebugData {
  int speedSetpoint;
  int speedTicksInput;
  int speedPwmOutput;
};

class MotorController {
  public:
    MotorController(float pGain, float iGain, float dGain);
    void setup(int enable, int enableB, int pwm1, int pwm2, int encoderA, int encoderB);
    void setLoopTime(int loopTime);
    void setControlLimits(int min, int max);
    void loop();
    void setDirection(int direction);
    void setSpeed(int speed);
    void setPwm(int pwm, int direction);
    void stop();
    void onEncoderTick();
    int update();
    int getDirection();
    MotorDebugData getDebugData();

  private:
    PID _speedPID;
    double _speedTicksInput;
    double _speedPwmOutput;
    double _speedSetpoint;

    int _pwm1Pin;
    int _pwm2Pin;
    int _pwmMinLimit = 0;
    int _pwmMaxLimit = 255;
    int _direction;
    float _lastError;
    float _accumulatedError;
    unsigned long _numTicks;
    unsigned long _numLastTicks;
    void motorControl(int pwm1, int pwm2);
};

#endif
