#include "PacketSerial.h"
#include "MotorController.h"

const int MOTOR_LEFT_ENABLE_PIN = 0;
const int MOTOR_LEFT_ENABLEB_PIN = 1;
const int MOTOR_LEFT_PWM1_PIN = 3;
const int MOTOR_LEFT_PWM2_PIN = 2;
const int MOTOR_LEFT_ENCODER_A_PIN = 17;
const int MOTOR_LEFT_ENCODER_B_PIN = 16;

const int MOTOR_RIGHT_ENABLE_PIN = 9;
const int MOTOR_RIGHT_ENABLEB_PIN = 10;
const int MOTOR_RIGHT_PWM1_PIN = 11;
const int MOTOR_RIGHT_PWM2_PIN = 12;
const int MOTOR_RIGHT_ENCODER_A_PIN = 14;
const int MOTOR_RIGHT_ENCODER_B_PIN = 15;

const byte REQUEST_START_FLAG = 0xA3;
const byte REQUEST_IS_READY = 0x01;
const byte REQUEST_SET_MOTOR_DIRECTIONS = 0x10;
const byte REQUEST_SET_MOTOR_SPEEDS = 0x11;
const byte REQUEST_SET_MOTOR_PWM = 0x12;
const byte REQUEST_MOTOR_STOP = 0x13;

const byte RESPONSE_START_FLAG_1 = 0xA3;
const byte RESPONSE_START_FLAG_2 = 0x3A;
const byte RESPONSE_ODOMETRY = 0x30;
const byte RESPONSE_READY = 0xFF;

const int pwmResolution = 13;
const int pwmControlLimit = 1023; // pow(2, pwmResolution) - 1;
const int loopTime = 20;
unsigned long previousTime = 0;

MotorController leftMotorController = MotorController(3, 90, 0.1);
MotorController rightMotorController = MotorController(3, 90, 0.1);

PacketSerial serial;

/**
 * Serial packet received event handler
 * @param {uint8_t} buffer
 * @param {size_t} size
 */
void onPacketReceived(const uint8_t* buffer, size_t size) {
  byte startFlag = buffer[0];
  byte command = buffer[1];

  if (startFlag == REQUEST_START_FLAG) {
    switch (command) {
      case REQUEST_SET_MOTOR_SPEEDS: {
        leftMotorController.setSpeed(buffer[2]);
        rightMotorController.setSpeed(buffer[3]);
        break;
      }

      case REQUEST_SET_MOTOR_PWM: {
        leftMotorController.setPwm((buffer[2] << 8) + buffer[3], buffer[4]);
        rightMotorController.setPwm((buffer[5] << 8) + buffer[6], buffer[7]);
        break;
      }

      case REQUEST_MOTOR_STOP: {
        leftMotorController.stop();
        rightMotorController.stop();
        break;
      }

      case REQUEST_IS_READY: {
        isReady();
        break;
      }

      case REQUEST_SET_MOTOR_DIRECTIONS: {
        leftMotorController.setDirection(buffer[2]);
        rightMotorController.setDirection(buffer[3]);
        break;
      }
    }
  }
}

/**
 * Send the ready response
 */
void isReady() {
  uint8_t readyResponse[4] = {
    RESPONSE_START_FLAG_1,
    RESPONSE_START_FLAG_2,
    RESPONSE_READY,
    0x00
  };

  serial.send(readyResponse, sizeof(readyResponse));
}

/**
 * Left encoder tick event handler
 */
void onLeftEncoderTick() {
  leftMotorController.onEncoderTick();
}

/**
 * Right encoder tick event handler
 */
void onRightEncoderTick() {
  rightMotorController.onEncoderTick();
}

/**
 * Setup
 */
void setup() {
  Serial.begin(115200);

  serial.setStream(&Serial);
  serial.setPacketHandler(&onPacketReceived);

  analogWriteResolution(pwmResolution);
  analogWriteFrequency(MOTOR_LEFT_PWM1_PIN, 18310.55);
  analogWriteFrequency(MOTOR_LEFT_PWM2_PIN, 18310.55);
  analogWriteFrequency(MOTOR_RIGHT_PWM1_PIN, 18310.55);
  analogWriteFrequency(MOTOR_RIGHT_PWM2_PIN, 18310.55);

  leftMotorController.setup(
    MOTOR_LEFT_ENABLE_PIN,
    MOTOR_LEFT_ENABLEB_PIN,
    MOTOR_LEFT_PWM1_PIN,
    MOTOR_LEFT_PWM2_PIN,
    MOTOR_LEFT_ENCODER_A_PIN,
    MOTOR_LEFT_ENCODER_B_PIN
  );

  leftMotorController.setControlLimits(0, pwmControlLimit);
  leftMotorController.setLoopTime(loopTime);

  rightMotorController.setup(
    MOTOR_RIGHT_ENABLE_PIN,
    MOTOR_RIGHT_ENABLEB_PIN,
    MOTOR_RIGHT_PWM1_PIN,
    MOTOR_RIGHT_PWM2_PIN,
    MOTOR_RIGHT_ENCODER_A_PIN,
    MOTOR_RIGHT_ENCODER_B_PIN
  );

  rightMotorController.setControlLimits(0, pwmControlLimit);
  rightMotorController.setLoopTime(loopTime);

  attachInterrupt(MOTOR_LEFT_ENCODER_A_PIN, onLeftEncoderTick, CHANGE);
  attachInterrupt(MOTOR_LEFT_ENCODER_B_PIN, onLeftEncoderTick, CHANGE);
  attachInterrupt(MOTOR_RIGHT_ENCODER_A_PIN, onRightEncoderTick, CHANGE);
  attachInterrupt(MOTOR_RIGHT_ENCODER_B_PIN, onRightEncoderTick, CHANGE);

  while (!Serial) {}

  isReady();

  previousTime = millis();
}

/**
 * Loop
 */
void loop() {
  serial.update();

  leftMotorController.loop();
  rightMotorController.loop();

  long now = millis();

  if (now - previousTime >= loopTime) {
    int leftMotorTicks = leftMotorController.update();
    int rightMotorTicks = rightMotorController.update();
    int leftMotorDirection = leftMotorController.getDirection();
    int rightMotorDirection = rightMotorController.getDirection();

    uint8_t response[8] = {
      RESPONSE_START_FLAG_1,
      RESPONSE_START_FLAG_2,
      RESPONSE_ODOMETRY,
      0x04,
      leftMotorTicks,
      leftMotorDirection,
      rightMotorTicks,
      rightMotorDirection
    };

    serial.send(response, sizeof(response));

    previousTime = now;
  }
}
