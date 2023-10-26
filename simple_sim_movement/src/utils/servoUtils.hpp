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
};

#endif /* SERVO_UTILS_HPP_ */