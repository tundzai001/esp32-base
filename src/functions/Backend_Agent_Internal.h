#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <Preferences.h>

// Shared implementation details for BackendAgent translation units.
namespace BackendAgentInternal {
inline constexpr size_t STATUS_DOCUMENT_SIZE = 8192;
inline constexpr size_t CONFIG_DOCUMENT_SIZE = 4096;
inline constexpr size_t COMMAND_DOCUMENT_SIZE = 24576;

Preferences &backendPreferences();
bool beginBackendPreferences(bool readOnly);
String readPreference(const char *key, const String &fallback = String());
String epochString();
bool jsonBool(JsonObjectConst object, const String &key, bool fallback);
String jsonString(JsonObjectConst object, const String &key, const String &fallback = String());
void copyJsonObject(JsonDocument &target, const char *key, const String &serialized);
String tokenFingerprint(const String &token);
size_t decodeBase64(const String &encoded, uint8_t *output, size_t outputCapacity);
bool delayMarker(const uint8_t *data, size_t length, uint32_t &delayMs);
bool licenseMatchesSerial(const String &serial, const String &token);
} // namespace BackendAgentInternal
