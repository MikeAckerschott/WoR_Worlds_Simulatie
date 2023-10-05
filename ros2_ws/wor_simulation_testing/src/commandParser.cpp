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
    // std::cout << "finding " << delimiter << " in " << messageString << std::endl;
    int iterator = messageString.find(delimiter);
    // std::cout << "found " << delimiter << " at " << iterator << std::endl;
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
    return parseIntAfterChar('P', isChannel, messageString);
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
    command.speed = getSpeed(messageString, isValid);
    if (!isValid && !usingDuration)
    {
        isValid = false;
        return command;
    }
    command.usingSpeed = isValid;

    isValid = true;
    return command;
}

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
        } else {
            command.servoCommands.clear();
            isValid = false;
            return command;
        }
    }

    isValid = true;
    return command;
}