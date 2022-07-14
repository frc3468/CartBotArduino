/*
  #include <SPI.h>
  #include <Ethernet.h>
  #include <EEPROM.h>
  #include <Servo.h>
*/

#include <RobotOpenHA.h>
//TODO WS2812B LED Strip Drive

#define ANALOG_RESOLUTION 1023 // 10-bit
#define PWM_NEUTRAL 127 // 0.0


// Controllers
const uint8_t driverControllerUSB = 1;

// PWM + Digital
// ARDUINO PINS 0 & 1 ARE RESERVED FOR THE SERIAL INTERFACE!!!
// ARDUINO PIN 10 IS RESERVED FOR THE ETHERNET CONTROLLER!!!!!
const uint8_t leftDriveMotorsPWM = 2;
const uint8_t rightDriveMotorsPWM = 3;

// Analog
const uint8_t voltageDividerAnalog = 0;


// I/O Objects
ROJoystick driverController(driverControllerUSB);

ROPWM leftDriveMotor(leftDriveMotorsPWM);
ROPWM rightDriveMotor(rightDriveMotorsPWM);

ROAnalog voltageDivider(voltageDividerAnalog);

// Drivetrain Variables
int8_t driveSpeedAxis() {
  return map(driverController.leftY(), 255, 0, -128, 127); /* inverted */
}
int8_t driveRotationAxis() {
  return map(driverController.leftX(), 0, 255, -128, 127);
}
const int8_t driveMinSpeed = -96;
const int8_t driveMaxSpeed = 96;

// Voltage Divider Variables
const uint8_t voltageDividerMaxReading = 25; // volts
const float voltageDividerStep = (float)voltageDividerMaxReading / ANALOG_RESOLUTION;


void setup()
{
  /* Initiate comms */
  RobotOpen.begin(&enabled, &disabled, &timedtasks);

  leftDriveMotor.attach();
  rightDriveMotor.attach();
}


/* This is your primary robot loop - all of your code
   should live here that allows the robot to operate
*/
void enabled() {
  // Drivetrain Control
  int8_t driveSpeed = driveSpeedAxis();
  int8_t driveRotation = driveRotationAxis();

  // Joystick Deadzones
  if (abs(driveSpeed) <= 10) {
    driveSpeed = 0;
  }
  if (abs(driveRotation) <= 10) {
    driveRotation = 0;
  }

  uint8_t leftDriveSpeed = map(constrain(driveSpeed + driveRotation, driveMinSpeed, driveMaxSpeed), -128, 127, 255, 0);
  uint8_t rightDriveSpeed = map(constrain(driveSpeed - driveRotation, driveMinSpeed, driveMaxSpeed), -128, 127, 0, 255);

  leftDriveMotor.write(leftDriveSpeed);
  rightDriveMotor.write(rightDriveSpeed);
}


/* This is called while the robot is disabled */
void disabled() {
  // safety code

  // Neutral-Out PWMs
  leftDriveMotor.write(PWM_NEUTRAL);
  rightDriveMotor.write(PWM_NEUTRAL);
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
