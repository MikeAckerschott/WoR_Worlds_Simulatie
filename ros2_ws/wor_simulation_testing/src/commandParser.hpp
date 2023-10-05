#include <string>
#include <vector>
#include <iostream>

class CommandParser
{
public:
    CommandParser();
    virtual ~CommandParser() = default;

    struct ServoCommand
    {
        int channel;
        int pulseWidth;
        int speed;
        bool usingSpeed;
    };

    struct CompleteCommand
    {
        std::vector<ServoCommand> servoCommands;
        int duration;
        bool usingDuration;
    };

    CommandParser::CompleteCommand parseCompleteCommand(std::string &messageString, bool &isValid);

private:
    int parseIntAfterChar(char delimiter, bool &isValid, std::string &messageString);
    int getChannel(std::string &messageString, bool &isChannel);
    int getPulseWidth(std::string &messageString, bool &isChannel);
    int getDuration(std::string &messageString, bool &isChannel);
    int getSpeed(std::string &messageString, bool &isChannel);

    std::vector<std::string> splitCommands(std::string &messageString);

    CommandParser::ServoCommand parseServoCommand(std::string &messageString, bool &isValid, bool &usingDuration);
};
