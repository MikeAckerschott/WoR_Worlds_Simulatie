/**
 * @file servoUtils.hpp
 * @brief Contains the ServoUtils class that provides helpful variables and functions for moving the robotic arm's servos safe and easy
 */

#ifndef SERVOUTILS_HPP_
#define SERVOUTILS_HPP_

#include <vector>
#include <iostream>

/**
 * @class ServoUtils
 * @brief Provides helpful variables and functions for moving the robotic arm's servos safe and easy
 */

class ServoUtils
{
public:
  /**
   * @brief Struct to store the range of motion for a servo
   * @param min The minimum angle the servo can move to
   * @param max The maximum angle the servo can move to
   */
  struct RangeOfMotion
  {
    int min;
    int max;
  };

  /**
   * @brief Struct to store the range of motion for the base of the robotic arm
   * @param min The minimum angle the base can move to: -90 degrees
   * @param max The maximum angle the base can move to: 90 degrees
   */
  static RangeOfMotion Base;

  /**
   * @brief Struct to store the range of motion for the shoulder of the robotic arm
   * @param min The minimum angle the shoulder can move to: -30 degrees
   * @param max The maximum angle the shoulder can move to: 90 degrees
   */
  static RangeOfMotion Shoulder;

  /**
   * @brief Struct to store the range of motion for the elbow of the robotic arm
   * @param min The minimum angle the elbow can move to: 0 degrees
   * @param max The maximum angle the elbow can move to: 135 degrees
   */
  static RangeOfMotion Elbow;

  /**
   * @brief Struct to store the range of motion for the wrist of the robotic arm
   * @param min The minimum angle the wrist can move to: -90 degrees
   * @param max The maximum angle the wrist can move to: 90 degrees
   */
  static RangeOfMotion Wrist;

  /**
   * @brief Struct to store the range of motion for the gripper of the robotic arm
   * @param min The minimum angle the gripper can move to: -60 degrees
   * @param max The maximum angle the gripper can move to: 60 degrees
   */
  static RangeOfMotion Gripper;

  /**
   * @brief Struct to store the range of motion for the wrist rotation of the robotic arm
   * @param min The minimum angle the wrist rotation can move to: -90 degrees
   * @param max The maximum angle the wrist rotation can move to: 90 degrees
   */
  static RangeOfMotion WristRotate;

  /**
   * @brief The minimum servo number: 0
   */
  static short minServo;

  /**
   * @brief The maximum servo number: 5
   */
  static short maxServo;

  /**
   * @brief The maximum speed in degrees per second a servo can move
   */
  static double maxSpeed;

  /**
   * @brief The max speed of the gripper in degrees per second
   */
  static double maxGripperSpeed;

  /**
   * @brief TF2 joint for left gripper
   */
  static short gripperRightTF2;

  /**
   * @brief TF2 joint for right gripper
   */
  static short gripperLeftTF2;

  /**
   * @brief the servo nr for the gripper on the AL5D
   */
  static short gripperServoAL5D;

  /**
   * @brief the servo nr for the wrist on the AL5D
   */
  static short wristServoAL5D;

  /**
   * @brief the servo nr for the wrist on the TF2
   */
  static short wristServoTF2;

  /**
   * @brief vector that stores the range of motion for each servo
   */
  static std::vector<RangeOfMotion> servoRanges;

  /**
   * @brief converts the pwm duration to degrees
   * @param pwmDurationUs The pwm duration in microseconds
   * @returns The degrees
   */
  static double pwmToDegrees(int pwmDurationUs, short servo);

  static double pwmToDegreesGripper(int pwmDurationUs);

  static double pwmPerSecondToDegreesPerSecond(int pwmPerSecond, short servo);

  static double getMaxSpeed(short servo);
  /**
   * @brief converts the degrees to pwm duration
   * @param servo The servo number
   * @param degrees The degrees to move to
   * @returns The pwm duration in microseconds
   */
  static double degreesToPwm(int servo, double degrees);

  /**
   * @brief verifies if the servo number and degrees are within the range of motion
   * @param servo The servo number
   * @param degrees The degrees to move to
   * @returns true if the servo number and degrees are within the range of motion, false if not
   */
  static bool verifyServoConstraints(short servo, double degrees);
};

#endif /* SERVO_UTILS_HPP_ */