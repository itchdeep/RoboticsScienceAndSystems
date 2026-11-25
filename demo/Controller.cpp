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

  // Exercise 7: spatial footprint. testFootprint handles its own button /
  // Serial trigger, settling delay, batch capture, beeps and zero motors.
  //(void)surface_ok;  // freshness is checked via the sensor timestamp instead
  //testFootprint(robot, now);

  //Previous button-toggled tests (offset / saturation):
  if (signal == 0) {
    if (robot.isButtonPressed()) {
      setSignal(1);
    }
  }

  if (signal == 1) {
    //runLineFollower(robot, now);
    if (surface_ok == true){
      testSaturation(robot, now);
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

void Controller_c::testSaturation(Robot_c &robot, unsigned long now) {
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
    Serial.print( "\n" );
  }
  
  // Report sensor readings to view on Serial Plotter
  for (unsigned long sensor = 0; sensor <= 4; sensor++) {
    Serial.print( "saturationTest");
    Serial.print( "," );
    Serial.print( 255 ); // surface_value
    Serial.print( "," );
    Serial.print( 1 );
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
  // Keep the motors commanded to zero on every cycle.
  robot.setMotorPWM(0, 0);

  // Trigger: the StampC3 button, or any character sent from the Serial Monitor
  // (lets you start a batch without touching the robot). Always drain Serial
  // so characters sent while busy don't queue up as extra triggers.
  bool trigger = robot.isButtonPressed();
  while (Serial.available() > 0) {
    Serial.read();
    trigger = true;
  }

  const bool strip_mode = (footprint_test == FOOTPRINT_STRIP);

  switch (fp_state) {

    case FP_IDLE: {
      if (!trigger) return;

      if (!fp_traverse_active) {
        // Start of a new independently registered traverse (strip mode), or a
        // new placement on a reference patch (reference modes).
        trial_num++;
        if (trial_num < footprint_first_trial) trial_num = footprint_first_trial;

        if (strip_mode) {
          float span = fabsf(footprint_end_mm - footprint_start_mm);
          fp_last_position_index = (int)lroundf(span / footprint_step_mm);
          fp_position_index = 0;
          position_mm = footprint_start_mm;
          fp_traverse_active = true;
        }

        // Header
        Serial.print("test_name,surface_value,trial_num,sample_num,sample_time,sensor_num,sensor_reading");
        if (strip_mode) {
          Serial.print(",rectangle_width_mm,direction,position_mm");
        }
        Serial.print("\n");
      }

      // New stop: sample_num counts fresh snapshots at this position, from 1.
      sample_num = 0;
      fp_settle_start_ms = now;
      fp_state = FP_SETTLING;
      robot.setLED(255, 120, 0, 60);   // orange: hands off, settling
      return;
    }

    case FP_SETTLING: {
      if (now - fp_settle_start_ms < footprint_settle_ms) return;

      // Ignore whatever snapshot was cached before settling finished.
      last_surface_us = robot.surface.timestamp_us;
      fp_state = FP_RECORDING;
      robot.setLED(255, 0, 0, 60);     // red: recording
      return;
    }

    case FP_RECORDING: {
      // Only log fresh snapshots. Use != rather than <= so the 32-bit
      // microsecond timestamp wrapping (~71 min) can't stall the logger.
      if (robot.surface.timestamp_us == last_surface_us) return;
      last_surface_us = robot.surface.timestamp_us;

      sample_num++;
      printFootprintSnapshot(robot);

      if (sample_num < footprint_samples_per_batch) return;

      // Batch complete.
      fp_state = FP_IDLE;

      if (strip_mode && fp_position_index < fp_last_position_index) {
        // Advance the label to the next stop; move the robot there next.
        fp_position_index++;
        float dir = (footprint_end_mm >= footprint_start_mm) ? 1.0f : -1.0f;
        position_mm = footprint_start_mm + dir * footprint_step_mm * fp_position_index;
        robot.playTone(1500, 100);     // short high beep: move to next position
        robot.setLED(0, 255, 0, 60);   // green: safe to move
      } else {
        // End of traverse (strip) or end of placement (reference).
        fp_traverse_active = false;
        robot.playTone(500, 600);      // long low beep: finished
        robot.setLED(0, 0, 255, 60);   // blue: done, next trigger = new trial
      }
      return;
    }
  }
}

void Controller_c::printFootprintSnapshot(Robot_c &robot) {
  const char* test_name;
  uint8_t surface_val;
  if (footprint_test == FOOTPRINT_WHITE_REF) {
    test_name = "footprintWhiteReference";
    surface_val = footprint_white_surface_value;
  } else if (footprint_test == FOOTPRINT_BLACK_REF) {
    test_name = "footprintBlackReference";
    surface_val = footprint_black_surface_value;
  } else {
    test_name = "footprintTest";
    surface_val = footprint_black_surface_value;  // nominal black target
  }

  const char* direction =
    (footprint_end_mm >= footprint_start_mm) ? "increasing" : "decreasing";

  // All five rows share trial, sample number, timestamp (and position).
  for (uint8_t sensor = 0; sensor < 5; sensor++) {
    Serial.print(test_name);
    Serial.print(",");
    Serial.print(surface_val);
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
    if (footprint_test == FOOTPRINT_STRIP) {
      Serial.print(",");
      Serial.print(rectangle_width_mm);
      Serial.print(",");
      Serial.print(direction);
      Serial.print(",");
      Serial.print(position_mm);
    }
    Serial.print("\n");
  }
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
