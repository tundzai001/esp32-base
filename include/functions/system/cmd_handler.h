#ifndef CMD_HANDLER_H
#define CMD_HANDLER_H

#include <Arduino.h>
#include <vector>

enum class cmd_action
{
    CMD_ACTION_NONE = 0,
    CMD_ACTION_ESP_RESTART = 1
};
using cmd_action_t = cmd_action;

std::vector<String> splitCommand(const String &command);

cmd_action_t handleCommand(std::vector<String> &cmdWords);

#endif // CMD_HANDLER_H