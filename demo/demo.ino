#include <Wire.h>

#include "RobotWifiAP.h"

/** @brief WiFi access-point and TCP server used for optional telemetry. */
RobotWifiAP_c server("tim", "timothy", 80);

#include "Robot.h"
#include "RobotIMU.h"
#include "Controller.h"
#include "TaskTimer.h"
TaskTimer_c LED_timer;
int led_brightness;

/** @brief Hardware-facing interface to the Pololu robot. */
Robot_c robot;
/** @brief Direct interface to the 3Pi+ accelerometer, gyro, and magnetometer. */
RobotIMU_c imu;
/** @brief Supplied nominal line-following controller. */
Controller_c controller;


/** @brief Initialises serial, WiFi, IMU, and the supplied robot-controller baseline. */
void setup() {

  Serial.begin(115200);
  delay(2000);
  Serial.println("Starting setup...");
  //server.begin();
  led_brightness = 0;

  //robot.initialise("segfault");
//  robot.initialise("team name");


  // Robot_c starts the shared I2C bus before RobotIMU_c uses it. IMU readings
  // come directly from the sensor ICs and do not pass through the middleware.
  if (!imu.initialise() ) {
    Serial.println("Warning: LSM6DS33 IMU was not detected.");
    robot.setLED( 90, 50, 0, 100 ); // orange warning
  }

  LED_timer.setIntervalMS(500);
  Serial.println("Setup reached the end!");

}


/** @brief Maintains telemetry transport and runs the controller repeatedly. */
void loop() {

  Serial.println("Hello from robot");
  delay(1000);
  // Accept or maintain the optional Processing telemetry connection. The
  // controller continues to run even when no client is connected.
  server.update();

  // Run an iteration of the robot controller code.
  // You shoul develop further code within Controller.h
  controller.update(robot, server);

}
