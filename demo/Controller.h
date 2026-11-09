#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "Robot.h"
#include "RobotWifiAP.h"
#include "TaskTimer.h"
#include "OdometryModel.h"

/**
 * @brief Controller with a selectable routine: parking, sensor data collection,
 * or the supplied line follower.
 *
 * Controller_c owns the controller state, while Robot_c owns the latest hardware
 * snapshot and RobotWifiAP_c owns optional telemetry transport. Students may read
 * or replace this controller, but should treat its constants as a demonstrator
 * baseline rather than a calibration of every physical robot.
 *
 * Choose what runs when the button is pressed by editing ACTIVE_ROUTINE below.
 * Pressing the button again while a routine is running stops the motors,
 * returns to waiting and increments trial_num, so you can run repeated trials.
 */
class Controller_c {
  
  public:
    /** @brief Routines that can be run after the button is pressed. */
    enum Routine {
      ROUTINE_PARK,            // drive forward and park in a black square
      ROUTINE_SENSOR_TEST,     // stream 5 sensor values (Serial Plotter), motors off
      ROUTINE_OFFSET_TEST,     // CSV sensor logging for the offset test, motors off
      ROUTINE_FOOTPRINT_TEST,  // CSV sensor logging for the footprint test, motors off
      ROUTINE_LINE_FOLLOW      // original supplied line follower
    };

    /** @brief >>> CHANGE THIS to select which routine runs. <<< */
    static const Routine ACTIVE_ROUTINE = ROUTINE_PARK;

    /** @brief Editable local encoder-to-pose model; inspect odometry.pose.
     * @note Reset automatically at the start of each run (see setSignal).
     */
    OdometryModel_c odometry;

    /** @brief Fastest interval between controller and I2C acquisition cycles. */
    static const unsigned long UPDATE_INTERVAL_MS = 10;
    /** @brief Interval between telemetry records in milliseconds. */
    static const unsigned long TELEMETRY_INTERVAL_MS = 50;

    // ---------------- Line follower parameters (original) ----------------
    /** @brief Maximum line-search duration before the controller stops. */
    static const unsigned long RECOVERY_TIMEOUT_MS = 5000;
    /** @brief Surface-reading threshold used to decide whether a line is present. */
    static const uint16_t LINE_THRESHOLD = 1400; // lower - thinks everything is a line; higher - thinks nothing is a line
    /** @brief Nominal forward PWM bias for line following. */
    static constexpr float BASE_PWM = 45.0f;
    /** @brief Proportional gain applied to the DN2-minus-DN4 surface error. */
    static constexpr float LINE_FOLLOW_GAIN = 0.01f;
    /** @brief Left PWM used during the one-direction line-search rotation. */
    static constexpr float RECOVERY_LEFT_PWM = -35.0f;
    /** @brief Right PWM used during the one-direction line-search rotation. */
    static constexpr float RECOVERY_RIGHT_PWM = 35.0f;

    // ---------------- Parking parameters ----------------
    /** @brief Surface reading at or above this counts as black (black~2000, white~400). */
    static const uint16_t BLACK_THRESHOLD = 1400;
    /** @brief Consecutive 10 ms cycles a reading must persist to be believed. */
    static const uint8_t CONFIRM_COUNT = 3;
    /** @brief Give up if black has not been found after this long. */
    static const unsigned long APPROACH_TIMEOUT_MS = 15000;
    /** @brief PWM while searching for the square. */
    static constexpr float APPROACH_PWM = 40.0f;
    /** @brief Slower PWM once inside, so the robot stops accurately. */
    static constexpr float PARK_PWM = 30.0f;
    /** @brief PWM correction per radian of heading error (keeps the path straight). */
    static constexpr float HEADING_GAIN = 40.0f;
    /** @brief Distance in mm from the wheel axle to DN3. */
    static constexpr float SENSOR_OFFSET_MM = 45.0f;
    /** @brief Side length of the black square in mm. MEASURE YOURS and change this. */
    static constexpr float SQUARE_SIDE_MM = 100.0f;
    /** @brief How far past the edge the robot centre should finish (default: centred). */
    static constexpr float PARK_DEPTH_MM = SQUARE_SIDE_MM / 2.0f;

    /** @brief Creates a controller in its waiting state. */
    Controller_c();

    /** @brief Clears signal and mode state, returning the controller to waiting.
     * @note This does not reset the task timers. It sends no motor command:
     * returning to waiting does not stop the motors. Send
     * robot.setMotorPWM(0, 0) separately and check the command result.
     */
    void reset();

    /**
     * @brief Returns whether the controller has received its start signal.
     * @return Zero while waiting; one after the button or caller starts it.
     */
    uint8_t getSignal() const;

    /**
     * @brief Starts the controller or resets it to waiting.
     * @param requested_signal Zero resets to waiting without sending a motor stop;
     * a non-zero value starts the active routine only if the current signal is zero.
     */
    void setSignal(uint8_t requested_signal);

    /**
     * @brief Runs one time-controlled acquisition, control, and telemetry cycle.
     * @param robot Hardware-facing interface and cached measurement snapshot.
     * @param server Optional WiFi/TCP telemetry transport.
     * @note Each accepted update cycle attempts to refresh sensors, encoders,
     * and the middleware pose. Failed reads retain their previous cached values;
     * local odometry is updated only after a successful encoder read. The method
     * returns early between update intervals.
     */
    void update(Robot_c &robot, RobotWifiAP_c &server);

    void testFootprint(Robot_c &robot, unsigned long now);

    // To track when the last sensor readings were retrieved in microseconds
    unsigned long last_surface_us;
    // Tracks how many sensor sample screenshots were recorded
    unsigned long sample_num;

    // csv test_name field
    const char* test_name_str = "footprintWhiteReference";
    // width of black test strip
    uint8_t rectangle_width_mm = 0;
    // signed position across strip in mm
    int16_t position_mm = 0;
    

  private:
    /** @brief Internal modes. The first four are used by the line follower. */
    enum Mode {
      WAITING,
      FOLLOWING_LINE,
      SEARCHING_FOR_LINE,
      STOPPED,
      APPROACHING,      // parking: driving over white, looking for the black edge
      PARKING,          // parking: inside the edge, driving the last distance
      PARKED,           // parking: success
      FAILED_TIMEOUT,   // parking: never found black
      FAILED_OVERRUN    // parking: ran out of black before reaching the target depth
    };

    uint8_t signal;
    Mode mode;
    TaskTimer_c update_timer;
    TaskTimer_c telemetry_timer;
    unsigned long recovery_start_ms;
    unsigned long trial_num = 0;

    // Parking state
    unsigned long approach_start_ms = 0;
    uint8_t black_count = 0;
    uint8_t white_count = 0;
    float edge_x = 0;
    float edge_y = 0;
    float heading_ref = 0;

    /** @brief Returns whether cached DN3 is at or above LINE_THRESHOLD. */
    bool lineDetected(const Robot_c &robot) const;

    /** @brief Returns whether a reading is at or above BLACK_THRESHOLD. */
    bool isBlack(uint16_t reading) const;

    /** @brief Drives forward at pwm while holding the starting heading. */
    void driveStraight(Robot_c &robot, float pwm);

    /** @brief Sends zero PWM to both motors. */
    void stopMotors(Robot_c &robot);

    /** @brief Drive forward and park in the black square (state machine). */
    void runParking(Robot_c &robot, unsigned long now);

    /** @brief sensor test function, prints IR readings to serial. */
    void sensorTest(Robot_c &robot, unsigned long now);

    /** @brief tests sensor offsets */
    void testOffset(Robot_c &robot, unsigned long now);

    /** @brief Applies the nominal controller behaviour for the current mode. */
    void runLineFollower(Robot_c &robot, unsigned long now);

    /**
     * @brief Sends the current controller and robot snapshot as one CSV record.
     * @note Controller.cpp writes these 14 columns, in order: timestamp_ms,
     * x, y, theta, left_pwm, right_pwm, left_count, right_count,
     * DN1, DN2, DN3, DN4, DN5, signal. These labels are not sent as a header.
     * x and y are in mm and theta is in radians, from the local Euler estimate,
     * not robot.pose from I2C. PWM values are cached requests, not measured output.
     * The timestamp is the controller-cycle start time; it does not guarantee
     * that every cached reading was successfully refreshed in that cycle.
     */
    void publishTelemetry(Robot_c &robot, RobotWifiAP_c &server, unsigned long timestamp_ms);
};

#endif