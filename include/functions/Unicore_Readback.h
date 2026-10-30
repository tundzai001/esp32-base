#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>

namespace UnicoreReadback {
bool readMode(String &mode, String &error);
bool readBestPosition(JsonObject position, String &error);
const char *modeKind(const String &mode);
} // namespace UnicoreReadback
