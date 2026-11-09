#include "Controller.h"
#include <math.h>

const unsigned long Controller_c::UPDATE_INTERVAL_MS;
const unsigned long Controller_c::TELEMETRY_INTERVAL_MS;
const unsigned long Controller_c::RECOVERY_TIMEOUT_MS;
const uint16_t Controller_c::LINE_THRESHOLD;
constexpr float Controller_c::BASE_PWM;
constexpr float Controller_c::LINE_FOLLOW_GAIN;
constexpr float Controller_c::RECOVERY_LEFT_PWM;
constexpr float Controller_c::RECOVERY_RIGHT_PWM;
const uint16_t Controller_c::BLACK_THRESHOLD;
const uint8_t Controller_c::CONFIRM_COUNT;
const unsigned long Controller_c::APPROACH_TIMEOUT_MS;
constexpr float Controller_c::APPROACH_PWM;
constexpr float Controller_c::PARK_PWM;
constexpr float Controller_c::HEADING_GAIN;
constexpr float Controller_c::SENSOR_OFFSET_MM;
constexpr float Controller_c::SQUARE_SIDE_MM;
constexpr float Controller_c::PARK_DEPTH_MM;

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

  approach_start_ms = 0;
  black_count = 0;
  white_count = 0;
  edge_x = 0;
  edge_y = 0;
  heading_ref = 0;
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
    recovery_start_ms = 0;

    // Each run starts a fresh odometry trial: origin (0,0), facing +x.
    // The robot is stationary when the button is pressed.
    odometry.reset(0.0f, 0.0f, 0.0f);
    heading_ref = 0.0f;

    if (ACTIVE_ROUTINE == ROUTINE_PARK) {
      mode = APPROACHING;
      approach_start_ms = 0;   // set on the first control cycle
      black_count = 0;
      white_count = 0;
    } else {
      mode = FOLLOWING_LINE;
    }
  }
}

void Controller_c::update(Robot_c &robot, RobotWifiAP_c &server) {

  // Get the current count of milliseconds since the StampC3 was powered on.
  unsigned long now = robot.getMillis();

  // Don't query the robot faster than every UPDATE_INTERVAL_MS.
  if (!update_timer.isReady(now)) {
    return;
  }
  update_timer.resetTimer(now);

  // Ask the robot for latest information on surface reflectance sensors.
  bool surface_ok = robot.getSurfaceSensors();

  // Latest encoders -> on-board odometry (see OdometryModel.h).
  if (robot.getEncoders()) {
    odometry.update(robot.getLeftEncoderCount(), robot.getRightEncoderCount());
  }

  // Pose estimation from the Pololu 3Pi robot itself.
  robot.getPose();

  // While waiting, a button press starts the selected routine. This is a start
  // control; the same button stops a running routine (handled below).
  if (signal == 0) {
    if (robot.isButtonPressed()) {
      setSignal(1);
    }
  }

  if (signal == 1) {

    switch (ACTIVE_ROUTINE) {

      case ROUTINE_PARK:
        runParking(robot, now);
        break;

      case ROUTINE_LINE_FOLLOW:
        runLineFollower(robot, now);
        break;

      case ROUTINE_SENSOR_TEST:
        sensorTest(robot, now);
        break;

      case ROUTINE_OFFSET_TEST:
        if (surface_ok == true) {
          testOffset(robot, now);
          sample_num++;
        }
        break;

      case ROUTINE_FOOTPRINT_TEST:
        if (surface_ok == true) {
          testFootprint(robot, now);
          sample_num++;
        }
        break;
    }

    // Button press while running: stop, return to waiting, count a new trial.
    // For parking this is also how you re-arm after the robot has parked.
    if (robot.isButtonPressed()) {
      stopMotors(robot);
      setSignal(0);
      trial_num++;
    }
  }

  // Telemetry is only sent for parking/line following, so it does not mix
  // with the CSV output of the data-collection routines.
  if (telemetry_timer.isReady(now)) {
    telemetry_timer.resetTimer(now);
    if (ACTIVE_ROUTINE == ROUTINE_PARK || ACTIVE_ROUTINE == ROUTINE_LINE_FOLLOW) {
      publishTelemetry(robot, server, now);
    }
  }
}

bool Controller_c::lineDetected(const Robot_c &robot) const {
  // Note: only using the central sensor to detect a line.
  if (robot.surface.reading[2] >= LINE_THRESHOLD) {
    return true;
  }

  return false;
}

bool Controller_c::isBlack(uint16_t reading) const {
  return reading >= BLACK_THRESHOLD;
}

void Controller_c::stopMotors(Robot_c &robot) {
  robot.setMotorPWM(0, 0);
}

// ======================= Data collection (unchanged) =======================

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
  for (uint8_t sensor = 0; sensor < 5; sensor++) {
    Serial.print(test_name_str);
    Serial.print(",");
    Serial.print(0);            // surface_value
    Serial.print(",");
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
    Serial.print(rectangle_width_mm);
    Serial.print(",");
    Serial.print("increasing");
    Serial.print(",");
    Serial.println(position_mm);
  }

  robot.setMotorPWM(0,0);
}

// ============================ Parking routine ============================

// Drive forward while holding the starting heading. Odometry theta is positive
// anticlockwise (left). If the robot has drifted left (theta > ref) we steer
// right: speed up the left wheel, slow the right.
void Controller_c::driveStraight(Robot_c &robot, float pwm) {
  float error = odometry.pose.theta - heading_ref;
  float correction = HEADING_GAIN * error;
  robot.setMotorPWM(pwm + correction, pwm - correction);
}

void Controller_c::runParking(Robot_c &robot, unsigned long now) {

  // DN3 (index 2) is the centre-front sensor, the first to reach the edge.
  bool dn3_black = isBlack(robot.surface.reading[2]);

  switch (mode) {

    case APPROACHING: {
      if (approach_start_ms == 0) {
        approach_start_ms = now;
      }

      if (now - approach_start_ms >= APPROACH_TIMEOUT_MS) {
        stopMotors(robot);
        mode = FAILED_TIMEOUT;
        robot.setLED(255, 0, 0, 60);   // red: failed
        return;
      }

      black_count = dn3_black ? (black_count < 255 ? black_count + 1 : 255) : 0;

      if (black_count >= CONFIRM_COUNT) {
        // Edge found. Remember where, then switch to the slower parking drive.
        edge_x = odometry.pose.x;
        edge_y = odometry.pose.y;
        white_count = 0;
        mode = PARKING;
        robot.setLED(0, 0, 255, 60);   // blue: inside the edge
        driveStraight(robot, PARK_PWM);
        return;
      }

      driveStraight(robot, APPROACH_PWM);
      break;
    }

    case PARKING: {
      // Distance travelled since DN3 crossed the edge.
      float dx = odometry.pose.x - edge_x;
      float dy = odometry.pose.y - edge_y;
      float travelled = sqrtf(dx * dx + dy * dy);

      // Robot centre is (travelled - SENSOR_OFFSET_MM) past the edge.
      float target = SENSOR_OFFSET_MM + PARK_DEPTH_MM;

      // Safety: DN3 back on white means the far edge has been reached.
      white_count = !dn3_black ? (white_count < 255 ? white_count + 1 : 255) : 0;
      if (white_count >= CONFIRM_COUNT) {
        stopMotors(robot);
        mode = FAILED_OVERRUN;
        robot.setLED(255, 0, 0, 60);
        return;
      }

      if (travelled >= target) {
        stopMotors(robot);
        mode = PARKED;
        robot.setLED(0, 255, 0, 60);   // green: parked
        return;
      }

      driveStraight(robot, PARK_PWM);
      break;
    }

    case PARKED:
    case FAILED_TIMEOUT:
    case FAILED_OVERRUN:
      stopMotors(robot);
      break;

    default:
      break;
  }
}

// ========================= Line follower (original) =========================

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