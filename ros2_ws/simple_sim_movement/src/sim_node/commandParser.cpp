#include "commandParser.hpp"

CommandParser::CommandParser()
{
}

std::vector<std::string> CommandParser::splitCommands(std::string &messageString)
{
    // # <ch> P <pw> S ​<spd>​ ... # <ch> P <pw> S <spd>​ T <time> <cr>
    std::vector<std::string> commands;
    int iterator = 0;

    // split string from first occurernce of # to the next occurence of # or end of string
    while (messageString[iterator] != '\0')
    {
        if (messageString[iterator] == '#')
        {
            std::string command = "";
            while (messageString[iterator] != '\0')
            {
                command += messageString[iterator];
                iterator++;
                if (messageString[iterator] == '#')
                {
                    --iterator;
                    break;
                }
            }
            commands.push_back(command);
        }
        iterator++;
    }

    return commands;
}

int CommandParser::parseIntAfterChar(char delimiter, bool &isValid, std::string &messageString)
{
    // this function ignores spaces
    int iterator = messageString.find(delimiter);
    if (iterator == -1)
    {
        isValid = false;
        return -1;
    }
    iterator++;
    isValid = false;
    std::string chString = "";

    while (messageString[iterator] != '\0')
    {
        if (messageString[iterator] == ' ')
        {
            iterator++;
            // std::cout<<"found space, skipping..."<<std::endl;
            continue;
        }
        if (messageString[iterator] < '0' || messageString[iterator] > '9')
        {
            // std::cout<<"found non-numeric value, breaking..."<<std::endl;
            break;
        }
        chString += messageString[iterator];
        iterator++;
    }

    if (messageString[iterator] == '\0' && chString.size() == 0)
    {
        isValid = false;
        return -1;
    }

    // get integer value from char found at iterator
    int channel = stoi(chString);
    isValid = true;
    return channel;
}

int CommandParser::getChannel(std::string &messageString, bool &isChannel)
{
    return parseIntAfterChar('#', isChannel, messageString);
}

int CommandParser::getPulseWidth(std::string &messageString, bool &isChannel)
{
    int pulseWidth = parseIntAfterChar('P', isChannel, messageString);
    isChannel = isChannel && pulseWidth >= 500 && pulseWidth <= 2500;
    return pulseWidth;
}

int CommandParser::getDuration(std::string &messageString, bool &isChannel)
{
    return parseIntAfterChar('T', isChannel, messageString);
}

int CommandParser::getSpeed(std::string &messageString, bool &isChannel)
{
    return parseIntAfterChar('S', isChannel, messageString);
}

CommandParser::ServoCommand CommandParser::parseServoCommand(std::string &messageString, bool &isValid, bool &usingDuration)
{
    ServoCommand command;
    isValid = false;
    command.channel = getChannel(messageString, isValid);
    if (!isValid)
    {
        return command;
    }
    command.pulseWidth = getPulseWidth(messageString, isValid);
    if (!isValid)
    {
        return command;
    }
    command.speedPWM = getSpeed(messageString, isValid);
    if (!isValid && !usingDuration)
    {
        return command;
    }
    command.usingSpeed = isValid;

    //translate AL5D channels to TF2 channels. 
    if (command.channel == ServoUtils::gripperServoAL5D)
    {
        command.channel = ServoUtils::gripperLeftTF2;
    }
    else if (command.channel == ServoUtils::wristServoAL5D)
    {
        command.channel = ServoUtils::wristServoTF2;
    }

    isValid = true;
    return command;
}

/**
 * parses command as al5d standards would allow
 *
 * REQ only 1 channel per command
 * REQ only 1 pulse width per command
 * REQ speed must be present per command if duration is not present
 * OPT speed is present per command
 * OPT duration is present
 */
CommandParser::CompleteCommand CommandParser::parseCompleteCommand(std::string &messageString, bool &isValid)
{
    CommandParser::CompleteCommand command;

    bool usingDuration = false;
    int duration = getDuration(messageString, usingDuration);

    if (usingDuration)
    {
        command.duration = duration;
        command.usingDuration = true;
    }
    else
    {
        command.duration = -1;
        command.usingDuration = false;
    }

    std::vector<std::string> servoCommands = splitCommands(messageString);

    for (std::string servoCommand : servoCommands)
    {
        bool isValid = true;
        auto parsedCommand = parseServoCommand(servoCommand, isValid, usingDuration);
        if (isValid)
        {
            command.servoCommands.push_back(parsedCommand);
            if (parsedCommand.channel == ServoUtils::gripperLeftTF2)
            {
                ServoCommand secondGripperCommand = parsedCommand;
                secondGripperCommand.channel = ServoUtils::gripperRightTF2;
                command.servoCommands.push_back(secondGripperCommand);
            }
        }
        else
        {
            std::cout << "Invalid command: " << servoCommand << std::endl;
            command.servoCommands.clear();
            isValid = false;
            return command;
        }
    }

    isValid = true;

    return command;
}

int CommandParser::calculateRealDuration(CompleteCommand &command, sensor_msgs::msg::JointState robotPositionMessage_)
{
    int realDuration = 0;
    if (command.usingDuration)
    {
        realDuration = command.duration;
    }

    // get the current position of all servos from topic /joint_states
    // calculate the time it takes to move from current position to desired position
    // if that time is greater than the current realDuration, set realDuration to that time

    for (size_t i = 0; i < command.servoCommands.size(); ++i)
    {
        int channel = command.servoCommands.at(i).channel;
        int pulseWidth = command.servoCommands.at(i).pulseWidth;
        double speed = command.servoCommands.at(i).speedPWM;

        double currentPosition = robotPositionMessage_.position[channel];

        double desiredPosAsDegrees = ServoUtils::pwmToDegrees(pulseWidth, channel);
        double currentPosAsDegrees = Utils::MathUtils::toDegrees(currentPosition);

        double distance = desiredPosAsDegrees - currentPosAsDegrees;

        double degreesPerSecond;
        if (command.servoCommands.at(i).usingSpeed)
        {
            degreesPerSecond = ServoUtils::pwmPerSecondToDegreesPerSecond(speed, channel);
            if (distance < 0)
            {
                degreesPerSecond *= -1;
            }
        }
        else
        {
            double durationAsSeconds = command.duration / 1000;
            degreesPerSecond = distance / durationAsSeconds;
        }

        if (degreesPerSecond > ServoUtils::getMaxSpeed(channel) || degreesPerSecond < -ServoUtils::getMaxSpeed(channel))
        {
            // set speed to max speed
            degreesPerSecond = std::min(std::max(degreesPerSecond, -ServoUtils::getMaxSpeed(channel)), ServoUtils::getMaxSpeed(channel));
        }

        command.servoCommands.at(i).speedAnglePerSecond = degreesPerSecond;

        std::cout << "Servo " << channel << " will move at " << degreesPerSecond << " degrees per second" << std::endl;

        double timeToMove = (distance / degreesPerSecond) * 1000; // in milliseconds

        if (timeToMove > realDuration)
        {
            realDuration = timeToMove;
        }
    }
    // set the data in the command and return it for easier access
    command.duration = realDuration;
    return realDuration;
}