#include <string>
#include <vector>
#include <iostream>

class CommandParser
{
public:
    CommandParser();
    virtual ~CommandParser() = default;

    int getChannel(std::string &messageString, bool &isChannel);
    int getPulseWidth(std::string &messageString, bool &isChannel);
    int getDuration(std::string &messageString, bool &isChannel);
    int getSpeed(std::string &messageString, bool &isChannel);

    std::vector<std::string> splitCommands(std::string &messageString);

private:
    int parseIntAfterChar(char delimiter, bool &isValid, std::string &messageString);
};
