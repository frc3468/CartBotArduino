/*
  #include <SPI.h>
  #include <Ethernet.h>
  #include <EEPROM.h>
  #include <Servo.h>
*/

#include <RobotOpenHA.h>

#define ANALOG_RESOLUTION 1023 // 10-bit

#define HW_MAX 255 // 1.0
#define HW_NEUTRAL 128 // 0.0
#define HW_MIN 0 //-1.0

#define SERVO_MIN 10
#define SERVO_MAX 245

#define SW_MAX 127 // 1.0
#define SW_NEUTRAL 0 // 0.0
#define SW_MIN -128//-1.0

// Controllers
const uint8_t driverControllerUSB = 1;

// PWM + Digital
// ARDUINO PINS 0 & 1 ARE RESERVED FOR THE SERIAL INTERFACE!!!
// ARDUINO PIN 10 IS RESERVED FOR THE ETHERNET CONTROLLER!!!!!
const uint8_t leftFrontDriveMotorsPWM = 2;
const uint8_t leftRearDriveMotorsPWM = 3;
const uint8_t rightFrontDriveMotorsPWM = 4;
const uint8_t rightRearDriveMotorsPWM = 5;

// Analog
const uint8_t voltageDividerAnalog = 0;


// I/O Objects
ROJoystick driverController(driverControllerUSB);

ROPWM leftFrontDriveMotor(leftFrontDriveMotorsPWM);
ROPWM leftRearDriveMotor(leftRearDriveMotorsPWM);
ROPWM rightFrontDriveMotor(rightFrontDriveMotorsPWM);
ROPWM rightRearDriveMotor(rightRearDriveMotorsPWM);

ROAnalog voltageDivider(voltageDividerAnalog);

// Drivetrain Variables
int8_t driveSpeedAxis() {
  return map(driverController.leftY(), HW_MIN, HW_MAX, SW_MAX, SW_MIN); /* inverted */
}
int8_t driveRotationAxis() {
  return map(driverController.leftX(), HW_MIN, HW_MAX, SW_MIN, SW_MAX);
}

const int8_t driveMaxSpeed = 50;
const int8_t driveMinSpeed = -1*driveMaxSpeed;
const uint8_t driveDeadzone = 10;

const uint8_t motorMaxSpeed = map(driveMaxSpeed, SW_MIN, SW_MAX, HW_MIN, HW_MAX);
const uint8_t motorMinSpeed = map(driveMinSpeed, SW_MIN, SW_MAX, HW_MIN, HW_MAX);

// Voltage Divider Variables
const uint8_t voltageDividerMaxReading = 25; // volts
const float voltageDividerStep = (float)voltageDividerMaxReading / ANALOG_RESOLUTION;


void setup()
{
  /* Initiate comms */
  RobotOpen.begin(&enabled, &disabled, &timedtasks);

  leftFrontDriveMotor.attach();
  leftRearDriveMotor.attach();
  rightFrontDriveMotor.attach();
  rightRearDriveMotor.attach();
}


/* This is your primary robot loop - all of your code
   should live here that allows the robot to operate
*/
void enabled() {
  // Drivetrain Control
  int8_t driveSpeed = driveSpeedAxis();
  int8_t driveRotation = driveRotationAxis();

  // Joystick Deadzones
  if (abs(driveSpeed) <= driveDeadzone) {
    driveSpeed = SW_NEUTRAL;
  }
  if (abs(driveRotation) <= driveDeadzone) {
    driveRotation = SW_NEUTRAL;
  }

  int8_t leftDriveSpeed = constrain(driveSpeed + driveRotation, SW_MIN, SW_MAX);
  int8_t rightDriveSpeed = constrain(driveSpeed - driveRotation, SW_MIN, SW_MAX);

  uint8_t leftMotorOutput = map(leftDriveSpeed,SW_MIN, SW_MAX, motorMaxSpeed, motorMinSpeed); /* Inverted */
  uint8_t rightMotorOutput = map(rightDriveSpeed, SW_MIN, SW_MAX, motorMinSpeed, motorMaxSpeed);
  
  leftFrontDriveMotor.write(leftMotorOutput);
  leftRearDriveMotor.write(leftMotorOutput);
  rightFrontDriveMotor.write(rightMotorOutput);
  rightRearDriveMotor.write(rightMotorOutput);
}


/* This is called while the robot is disabled */
void disabled() {
  // safety code

  // Neutral-Out PWMs
  leftFrontDriveMotor.write(HW_NEUTRAL);
  leftRearDriveMotor.write(HW_NEUTRAL);
  rightFrontDriveMotor.write(HW_NEUTRAL);
  rightRearDriveMotor.write(HW_NEUTRAL);
}


/* This loop ALWAYS runs - only place code here that can run during a disabled state
   This is also a good spot to put driver station publish code
*/
void timedtasks() {
  RODashboard.publish("Uptime (s)", ROStatus.uptimeSeconds());
  RODashboard.publish("Batt Voltage (v)", (voltageDivider.read()*voltageDividerStep));
}


// !!! DO NOT MODIFY !!!
void loop() {
  RobotOpen.syncDS();
}
