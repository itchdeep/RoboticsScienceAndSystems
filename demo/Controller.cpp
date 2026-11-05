#include "Controller.h"

const unsigned long Controller_c::UPDATE_INTERVAL_MS;
const unsigned long Controller_c::TELEMETRY_INTERVAL_MS;
const unsigned long Controller_c::RECOVERY_TIMEOUT_MS;
const uint16_t Controller_c::LINE_THRESHOLD;
constexpr float Controller_c::BASE_PWM;
constexpr float Controller_c::LINE_FOLLOW_GAIN;
constexpr float Controller_c::RECOVERY_LEFT_PWM;
constexpr float Controller_c::RECOVERY_RIGHT_PWM;

Controller_c::Controller_c()
  : update_timer(UPDATE_INTERVAL_MS),
    telemetry_timer(TELEMETRY_INTERVAL_MS) {
  reset();
}

void Controller_c::reset() {
  signal = 0;
  mode = WAITING;
  recovery_start_ms = 0;
  last_surface_us = 0;
  sample_num = 0;
}

uint8_t Controller_c::getSignal() const {
  return signal;
}

void Controller_c::setSignal(uint8_t requested_signal) {
  if (requested_signal == 0) {
    reset();
    return;
  }

  if (signal == 0) {
    signal = 1;
    mode = FOLLOWING_LINE;
    recovery_start_ms = 0;
  }
}

void Controller_c::update(Robot_c &robot, RobotWifiAP_c &server) {

  // Get the current count of milliseconds since the StampC3 was powered on.
  unsigned long now = robot.getMillis();

  // Check if our TaskTimer_c for the general controller update indicates
  // that it is the correct time to operate.  Note, we give the TaskTimer_c
  // the current count in milliseconds.
  // We do this because we don't want to query the robot faster than at 10ms
  // intervals.
  if (!update_timer.isReady(now)) {

    // TaskTimer_c says it is not ready, so we simply end this call to
    // the controller here by calling return.
    return;
  }

  // The TaskTimer_c was ready, so we now reset the TaskTimer_c so it
  // begins counting again.  We now run the remaining controller code.
  update_timer.resetTimer(now);

  // Ask the robot for latest information on surface reflectance sensors.
  bool surface_ok = robot.getSurfaceSensors();

  // Ask the robot for the latest encoder information, and then use this
  // to update the odometry on board the StampC3 (see OdometryModel.h).
  if (robot.getEncoders()) {
    odometry.update(robot.getLeftEncoderCount(), robot.getRightEncoderCount());
  }

  // Pose estimation from the Pololu 3Pi robot itself.
  robot.getPose();

  

  // While waiting, a button press sets signal to 1 and starts the physical
  // robot's line follower, even without Processing connected. The transmitted
  // signal also starts an armed Digital Twin live comparison. This is a start
  // control, not a start/stop toggle; further presses are ignored while signal is 1.
  if (signal == 0) {
    if (robot.isButtonPressed()) {
      setSignal(1);
    }
  }

  // The user has started the demonstration, so we run the line following
  // controller code.
  if (signal == 1) {
    //runLineFollower(robot, now);
    if (surface_ok == true){
      testOffset(robot, now);
      sample_num++;
    }
    if (robot.isButtonPressed()) {
      setSignal(0);
      trial_num++;
    }
  }

  // Use another TaskTimer_c to limit how often we transmit telemetry data
  // on WiFi and Serial.
  if (telemetry_timer.isReady(now)) {
    telemetry_timer.resetTimer(now);
    //publishTelemetry(robot, server, now);
  }
}

bool Controller_c::lineDetected(const Robot_c &robot) const {
  // Note: only using the central sensor to detect a line.
  if (robot.surface.reading[2] >= LINE_THRESHOLD) {
    return true;
  }

  return false;
}

void Controller_c::sensorTest(Robot_c &robot, unsigned long now) {
  // Report sensor readings to view on Serial Plotter
  Serial.print( robot.surface.reading[0] ); // DN1
  Serial.print( "," );
  Serial.print( robot.surface.reading[1] ); // DN2
  Serial.print( "," );
  Serial.print( robot.surface.reading[2] ); // DN3
  Serial.print( "," );
  Serial.print( robot.surface.reading[3] ); // DN4
  Serial.print( "," );
  Serial.print( robot.surface.reading[4] ); // DN5
  Serial.print( "\n" );

  // Ensure the robot is stopped
  robot.setMotorPWM(0,0);
}

void Controller_c::testOffset(Robot_c &robot, unsigned long now) {
  // check if last reading timestamp has advanced
  if (robot.surface.timestamp_us <= last_surface_us){
    return;
  }
  last_surface_us = robot.surface.timestamp_us;

  if (sample_num == 0) {
    // Header
    Serial.print( "test_name" );
    Serial.print( "," );
    Serial.print( "trial_num" );
    Serial.print( "," );
    Serial.print( "sample_num" );
    Serial.print( "," );
    Serial.print( "sample_time" );
    Serial.print( "," );
    Serial.print( "sensor_num" );
    Serial.print( "," );
    Serial.print( "sensor_reading" );
    Serial.print( "\n" );
  }
  
  // Report sensor readings to view on Serial Plotter
  for (unsigned long sensor = 0; sensor <= 4; sensor++) {
    Serial.print( "offsetTest_white");
    Serial.print( "," );
    Serial.print( trial_num );
    Serial.print( "," );
    Serial.print( sample_num );
    Serial.print( "," );
    Serial.print( robot.surface.timestamp_us );
    Serial.print( "," );
    Serial.print( sensor );
    Serial.print( "," );
    Serial.print( robot.surface.reading[sensor] );
    Serial.print( "\n" );
  }
  // Ensure the robot is stopped
  robot.setMotorPWM(0,0);
}

void Controller_c::testFootprint(Robot_c &robot, unsigned long now) {
  // check if last reading timestamp has advanced
  if (robot.surface.timestamp_us <= last_surface_us){
    return;
  }
  last_surface_us = robot.surface.timestamp_us;

  if (sample_num == 0) {
    // Header
    Serial.print( "test_name" );
    Serial.print( "," );
    Serial.print( "surface_value" );
    Serial.print( "," );
    Serial.print( "trial_num" );
    Serial.print( "," );
    Serial.print( "sample_num" );
    Serial.print( "," );
    Serial.print( "sample_time" );
    Serial.print( "," );
    Serial.print( "sensor_num" );
    Serial.print( "," );
    Serial.print( "sensor_reading" );
    Serial.print( "," );
    Serial.print( "rectangle_width_mm" );
    Serial.print( "," );
    Serial.print( "direction" );
    Serial.print( "," );
    Serial.print( "position_mm" );
    Serial.print( "\n" );
  }

  // Report readings for all 5 surface sensors (DN1 to DN5) 
  for (uint8\_t sensor = 0; sensor &lt; 5; sensor++) {
    Serial.print(test_name_str);
    Serial.print(",");
    Serial.print(0); 
    Serial.print(","); // surface\_value 
    Serial.print(trial_num);
    Serial.print(",");
    Serial.print(sample_num);
    Serial.print(",");
    Serial.print(robot.surface.timestamp_us);
    Serial.print(",");
    Serial.print(sensor);
    Serial.print(",");
    Serial.print(robot.surface.reading[sensor]);
    Serial.print(",");
    Serial.print(rectangle\_width\_mm);
    Serial.print(",");
    Serial.print("increasing");
    Serial.print(",");
    Serial.println(position\_mm);
  }

  robot.setMotorPWM(0,0);
}

void Controller_c::runLineFollower(Robot_c &robot, unsigned long now) {
  bool line_detected = lineDetected(robot);

  if (!line_detected && mode == FOLLOWING_LINE) {
    mode = SEARCHING_FOR_LINE;
    recovery_start_ms = now;
  }

  if (mode == SEARCHING_FOR_LINE) {
    if (line_detected) {
      mode = FOLLOWING_LINE;
    } else if (now - recovery_start_ms >= RECOVERY_TIMEOUT_MS) {
      mode = STOPPED;
    }
  }

  if (mode == STOPPED) {
    robot.setMotorPWM(0, 0);
    return;
  }

  if (mode == SEARCHING_FOR_LINE) {
    robot.setMotorPWM(RECOVERY_LEFT_PWM, RECOVERY_RIGHT_PWM);
    return;
  }

  float error = (float)robot.surface.reading[1] - (float)robot.surface.reading[3];
  float turn = error * LINE_FOLLOW_GAIN;
  robot.setMotorPWM(BASE_PWM - turn, BASE_PWM + turn);
}

void Controller_c::publishTelemetry(Robot_c &robot, RobotWifiAP_c &server, unsigned long timestamp_ms) {
  server.printf(
    "%lu,%.2f,%.2f,%.5f,%d,%d,%ld,%ld,%u,%u,%u,%u,%u,%u\n",
    timestamp_ms,
    odometry.pose.x,
    odometry.pose.y,
    odometry.pose.theta,
    robot.getLeftMotorPWM(),
    robot.getRightMotorPWM(),
    (long)robot.getLeftEncoderCount(),
    (long)robot.getRightEncoderCount(),
    robot.surface.reading[0],
    robot.surface.reading[1],
    robot.surface.reading[2],
    robot.surface.reading[3],
    robot.surface.reading[4],
    signal
  );
  if ( Serial ) { // If serial is connected
    Serial.printf(
      "%lu,%.2f,%.2f,%.5f,%d,%d,%ld,%ld,%u,%u,%u,%u,%u,%u\n",
      timestamp_ms,
      odometry.pose.x,
      odometry.pose.y,
      odometry.pose.theta,
      robot.getLeftMotorPWM(),
      robot.getRightMotorPWM(),
      (long)robot.getLeftEncoderCount(),
      (long)robot.getRightEncoderCount(),
      robot.surface.reading[0],
      robot.surface.reading[1],
      robot.surface.reading[2],
      robot.surface.reading[3],
      robot.surface.reading[4],
      signal
    );
  }
}
