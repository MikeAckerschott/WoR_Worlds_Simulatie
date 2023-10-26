#include "servoUtils.hpp"

ServoUtils::RangeOfMotion ServoUtils::Base = {-90, 90};
ServoUtils::RangeOfMotion ServoUtils::Shoulder = {-30, 90};
ServoUtils::RangeOfMotion ServoUtils::Elbow = {0, 135};
ServoUtils::RangeOfMotion ServoUtils::Wrist = {-90, 90};
ServoUtils::RangeOfMotion ServoUtils::Gripper = {-60, 60};
ServoUtils::RangeOfMotion ServoUtils::WristRotate = {-90, 90};

short ServoUtils::minServo = 0;
short ServoUtils::maxServo = 5;
double ServoUtils::maxSpeed = 360;
double ServoUtils::maxGripperSpeed = 1.7;

short ServoUtils::gripperLeftTF2 = 5;
short ServoUtils::gripperRightTF2 = 6;
short ServoUtils::gripperServoAL5D = 4;

short ServoUtils::wristServoAL5D = 5;
short ServoUtils::wristServoTF2 = 4;

double ServoUtils::pwmToDegrees(int pwmDurationUs, short servo)
{
    // Define the PWM-to-degree mapping parameters
    const int minPwmUs = 500;       // Minimum PWM duration for -90 degrees
    const double maxPwmUs = 2500.0; // Maximum PWM duration for +90 degrees
    const int centerPwmUs = 1500;   // Center position corresponds to 0 degrees

    if (servo == ServoUtils::gripperLeftTF2 || servo == ServoUtils::gripperRightTF2)
    {
        return pwmToDegreesGripper(pwmDurationUs);
    }

    // Calculate the degrees using linear interpolation
    double degrees = static_cast<double>(pwmDurationUs - centerPwmUs) / (centerPwmUs - minPwmUs) * 90.0;
    return degrees;
}

double ServoUtils::pwmToDegreesGripper(int pwmDurationUs)
{
    // Define the PWM-to-degree mapping parameters
    const int minPwmUs = 500;               // Minimum PWM duration for -90 degrees
    const double maxPwmUs = 2500.0;         // Maximum PWM duration for +90 degrees
    const int centerPwmUs = 1500;           // Center position corresponds to 0 degrees
    const double pwmGripperOffset = 2650.0; // Offset for gripper pwm, causes pwm of 500 to be fully open and 2500 to be fully closed

    pwmDurationUs -= minPwmUs; // pwm of 500 should be 0 degrees, or fully open
    pwmDurationUs = pwmDurationUs / maxPwmUs * 2650.0;
    double degrees = pwmDurationUs / maxPwmUs;
    return degrees;
}

double ServoUtils::pwmPerSecondToDegreesPerSecond(int pwmPerSecond, short servo)
{
    // Define the PWM-to-degree mapping parameters
    const int minPwmUs = 500;       // Minimum PWM duration for -90 degrees
    const double maxPwmUs = 2500.0; // Maximum PWM duration for +90 degrees
    const int centerPwmUs = 1500;   // Center position corresponds to 0 degrees

    if (servo == gripperLeftTF2 || servo == gripperRightTF2)
    {
        return pwmToDegrees(pwmPerSecond + minPwmUs, servo); // minPwmUs (500) is fully open, maxPwmUs (2500) is fully closed
    }
    else
    {
        return pwmToDegrees(pwmPerSecond + 1500, servo); // 1500 is the center position (0 degrees)
    }
}

bool ServoUtils::verifyServoConstraints(short servo, double degrees)
{
    if (servo < minServo || servo > maxServo)
    {
        return false;
    }

    switch (servo)
    {
    case 0:
        if (degrees < Base.min || degrees > Base.max)
        {
            return false;
        }
        break;
    case 1:
        if (degrees < Shoulder.min || degrees > Shoulder.max)
        {
            return false;
        }
        break;
    case 2:
        if (degrees < Elbow.min || degrees > Elbow.max)
        {
            return false;
        }
        break;
    case 3:
        if (degrees < Wrist.min || degrees > Wrist.max)
        {
            return false;
        }
        break;
    case 4:
        if (degrees < Gripper.min || degrees > Gripper.max)
        {
            return false;
        }
        break;
    case 5:
        if (degrees < WristRotate.min || degrees > WristRotate.max)
        {
            return false;
        }
        break;
    default:
        return false;
    }

    return true;
}

double ServoUtils::degreesToPwm(int servo, double degrees)
{

    // Define the PWM-to-degree mapping parameters
    const int minPwmUs = 500; // Minimum PWM duration for -90 degrees
    // const int maxPwmUs = 2500;    // Maximum PWM duration for +90 degrees. although not used, it is here for reference
    const int centerPwmUs = 1500; // Center position corresponds to 0 degrees

    // Calculate the PWM duration using linear interpolation
    double pwmDurationUs = static_cast<double>(degrees) / 90.0 *
                               (centerPwmUs - minPwmUs) +
                           centerPwmUs;

    if (servo == 2) // degrees 0 should be straight up (600 pwm)
    {
        pwmDurationUs = pwmDurationUs - 900;
    }
    return pwmDurationUs;
}

double ServoUtils::getMaxSpeed(short servo)
{
    if (servo == gripperLeftTF2 || servo == gripperRightTF2)
    {
        return maxGripperSpeed;
    }
    else
    {
        return maxSpeed;
    }
}
