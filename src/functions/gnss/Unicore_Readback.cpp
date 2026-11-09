#include "functions/gnss/Unicore_Readback.h"
#include "functions/gnss/Unicore_Reply.h"
#include "functions/gnss/Unicore_Log.h"
#include "functions/ntrip/RTCM_Receiver.h"
#include "Prog_Config.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <array>
#include <cmath>
#include <cstdlib>

extern SemaphoreHandle_t rtcmBufferMutex;
extern SemaphoreHandle_t gnssSerialMutex;

namespace UnicoreReadback {
namespace {
bool finishMultilineCandidate(char value, const char *prefix,
                              String &candidate, String &response) {
    if (!candidate.startsWith(prefix)) return false;

    candidate += value;
    if (candidate.endsWith("\r\n\r\n") || candidate.endsWith("\n\n")) {
        response = candidate;
        return true;
    }

    auto end = candidate.length();
    while (end && (candidate[end - 1] == '\r' || candidate[end - 1] == '\n'
                   || candidate[end - 1] == ' ')) {
        --end;
    }
    if (end >= 9 && candidate[end - 9] == '*') {
        candidate.remove(end);
        response = candidate;
        return true;
    }
    return false;
}

bool finishUnicoreCandidate(char value, const char *prefix, bool multiline,
                            String &candidate, String &response) {
    if (multiline && candidate.startsWith(prefix)) {
        return finishMultilineCandidate(value, prefix, candidate, response);
    }
    if (candidate.startsWith(prefix)) {
        response = candidate;
        return true;
    }
    candidate = "";
    return false;
}

bool appendUnicoreCandidate(char value, const char *prefix, bool multiline,
                            String &candidate, String &response) {
    if (value == prefix[0]) {
        candidate = String(value);
        return false;
    }
    if (multiline && value == '\r' && !candidate.isEmpty()) {
        candidate += value;
        return false;
    }
    if (value == '\r' || value == '\n') {
        return finishUnicoreCandidate(value, prefix, multiline, candidate, response);
    }
    if (candidate.isEmpty()) return false;

    
    if (const auto limit = multiline ? 4095U : 383U;
        candidate.length() < limit)
    {
        candidate += value;
    } else {
        candidate = "";
    }
    return false;
}

void processUnicoreResponseByte(char value, const char *prefix, bool multiline,
                                UnicoreReply &reply, String &candidate,
                                String &response, String &error) {
    bufferGnssQueryByte(static_cast<uint8_t>(value));
    if (reply.feed(static_cast<uint8_t>(value))) {
        Serial.printf("[GNSS][READBACK][ACK] %s\n", reply.line);
        if (reply.state == UnicoreReply::State::Error) {
            error = String("Unicore rejected query: ") + reply.line;
            return;
        }
    }
    appendUnicoreCandidate(value, prefix, multiline, candidate, response);
}

bool queryUnicoreLine(const char *query, const char *prefix, String &response, String &error,
                      bool multiline = false) {
    if (!rtcmBufferMutex
        || xSemaphoreTake(rtcmBufferMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) != pdTRUE) {
        error = "GNSS command port is busy";
        return false;
    }
    if (!gnssSerialMutex
        || xSemaphoreTake(gnssSerialMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) != pdTRUE) {
        xSemaphoreGive(rtcmBufferMutex);
        error = "GNSS reader is busy";
        return false;
    }

    while (Serial1.available() > 0) bufferGnssQueryByte(static_cast<uint8_t>(Serial1.read()));
    String command(query);
    command += "\r\n";
    UnicoreReply reply(reinterpret_cast<const uint8_t *>(command.c_str()), command.length());
    Serial.printf("[GNSS][READBACK][TX] %s; baud=%lu\n", query,
                  static_cast<unsigned long>(Serial1.baudRate()));
    // Finish any partial input line left on the mixed binary/ASCII command port.
    const bool written = Serial1.write(reinterpret_cast<const uint8_t *>("\r\n"), 2) == 2
        && Serial1.write(
            reinterpret_cast<const uint8_t *>(command.c_str()), command.length()) == command.length();
    Serial1.flush();
    String candidate;
    candidate.reserve(160);
    uint32_t receivedBytes = 0;
    const uint32_t started = millis();
    while (written && millis() - started < 3000UL && response.isEmpty() && error.isEmpty()) {
        while (Serial1.available() > 0 && response.isEmpty() && error.isEmpty()) {
            const auto value = static_cast<char>(Serial1.read());
            ++receivedBytes;
            processUnicoreResponseByte(value, prefix, multiline, reply, candidate, response, error);
        }
        if (response.isEmpty()) vTaskDelay(pdMS_TO_TICKS(5));
    }
    xSemaphoreGive(gnssSerialMutex);
    xSemaphoreGive(rtcmBufferMutex);

    Serial.printf("[GNSS][READBACK][RX] query=%s; bytes=%lu; elapsed_ms=%lu; reply=%u\n",
                  query, static_cast<unsigned long>(receivedBytes),
                  millis() - started, static_cast<unsigned int>(reply.state));

    if (!error.isEmpty()) return false;
    if (!written) {
        error = String("Unicore ") + query + " query write failed";
        return false;
    }
    if (response.isEmpty()) {
        error = String("Unicore ") + query + " readback timeout";
        return false;
    }
    return true;
}

bool splitBestPositionPayload(const String &payload, std::array<String, 10> &fields, String &error) {
    auto start = 0;
    for (size_t index = 0; index < 10; ++index) {
        const auto comma = payload.indexOf(',', start);
        if (index < 6 && comma < 0) {
            error = "Unicore BESTNAV response is incomplete";
            return false;
        }
        const auto end = comma < 0 ? payload.length() : comma;
        fields[index] = payload.substring(start, end);
        start = end + 1;
        if (comma < 0) break;
    }
    for (String &field : fields) field.trim();
    return true;
}

bool parseBestCoordinate(const String &value, const char *name, double &result, String &error) {
    char *end = nullptr;
    result = std::strtod(value.c_str(), &end);
    if (end != value.c_str() && *end == '\0') return true;
    error = String("Unicore BESTNAV ") + name + " is invalid";
    return false;
}

bool parseBestPositionCoordinates(const std::array<String, 10> &fields, JsonObject position, String &error) {
    if (!fields[0].equalsIgnoreCase("SOL_COMPUTED")
        || fields[2].isEmpty() || fields[3].isEmpty() || fields[4].isEmpty()) {
        error = String("Unicore BESTNAV has no computed position: ") + fields[0];
        return false;
    }

    double latitude = 0.0;
    double longitude = 0.0;
    double height = 0.0;
    if (!parseBestCoordinate(fields[2], "latitude", latitude, error)
        || !parseBestCoordinate(fields[3], "longitude", longitude, error)
        || !parseBestCoordinate(fields[4], "height", height, error)) {
        return false;
    }
    if (!std::isfinite(latitude) || !std::isfinite(longitude) || !std::isfinite(height)
        || latitude < -90.0 || latitude > 90.0
        || longitude < -180.0 || longitude > 180.0) {
        error = "Unicore BESTNAV coordinates are invalid";
        return false;
    }
    position["state"] = "ok";
    position["latitude"] = latitude;
    position["longitude"] = longitude;
    position["height_msl"] = height;
    return true;
}

bool parseBestUncertainty(const std::array<String, 10> &fields, JsonObject position, String &error) {
    char *end = nullptr;
    const double undulation = std::strtod(fields[5].c_str(), &end);
    if (end == fields[5].c_str() || *end != '\0'
        || !std::isfinite(undulation) || fields[6] != "WGS84") {
        error = "Unicore BESTNAV undulation/datum is invalid";
        return false;
    }
    position["undulation"] = undulation;
    if (fields[7].isEmpty() || fields[8].isEmpty()) return true;

    const double latSigma = std::strtod(fields[7].c_str(), &end);
    const bool latValid = end != fields[7].c_str() && *end == '\0'
        && std::isfinite(latSigma) && latSigma >= 0 && latSigma <= 10000000;
    const double lonSigma = std::strtod(fields[8].c_str(), &end);
    const bool lonValid = end != fields[8].c_str() && *end == '\0'
        && std::isfinite(lonSigma) && lonSigma >= 0 && lonSigma <= 10000000;
    if (latValid && lonValid) position["horizontal_sigma_m"] = std::hypot(latSigma, lonSigma);
    return true;
}

} // namespace

bool readLogList(String &response, String &error) {
    if (!queryUnicoreLine("UNILOGLIST", "#UNILOGLIST,", response, error, true)) return false;
    if (response.indexOf('*') >= 0 && !validUnicoreLogChecksum(response.c_str(), response.length())) {
        error = "Unicore UNILOGLIST checksum mismatch";
        return false;
    }
    return true;
}

bool readMode(String &mode, String &error) {
    String response;
    if (!queryUnicoreLine("MODE", "#MODE", response, error)) {
        Serial.printf("[GNSS][READBACK][ERROR] %s\n", error.c_str());
        return false;
    }
    Serial.printf("[GNSS][READBACK] %s\n", response.c_str());
    if (!validUnicoreLogChecksum(response.c_str(), response.length())) {
        error = "Unicore MODE checksum mismatch";
        Serial.printf("[GNSS][READBACK][ERROR] %s\n", error.c_str());
        return false;
    }
    const int separator = response.indexOf(';');
    if (separator < 0) {
        error = "Unicore MODE response is malformed";
        return false;
    }
    mode = response.substring(separator + 1);
    if (const int checksum = mode.indexOf('*'); checksum >= 0)
        mode.remove(checksum);

    mode.trim();
    if (mode.endsWith(",")) mode.remove(mode.length() - 1);
    mode.toUpperCase();
    if (!mode.startsWith("MODE ")) {
        error = "Unicore MODE response has no operating mode";
        mode = "";
        return false;
    }
    return true;
}

bool readBestPosition(JsonObject position, String &error) {
    String response;
    if (!queryUnicoreLine("BESTNAVA", "#BESTNAVA,", response, error)) return false;
    if (!validUnicoreLogChecksum(response.c_str(), response.length())) {
        error = "Unicore BESTNAV checksum mismatch";
        return false;
    }
    position["message"] = "BESTNAV";
    position["raw"] = response;
    const int separator = response.indexOf(';');
    if (separator < 0) {
        error = "Unicore BESTNAV response is malformed";
        return false;
    }
    String payload = response.substring(separator + 1);
    if (const int checksum = payload.indexOf('*'); checksum >= 0)
        payload.remove(checksum);
    std::array<String, 10> fields;
    if (!splitBestPositionPayload(payload, fields, error)) return false;
    position["solution_status"] = fields[0];
    position["position_type"] = fields[1];
    return parseBestPositionCoordinates(fields, position, error)
        && parseBestUncertainty(fields, position, error);
}

const char *modeKind(const String &mode) {
    if (mode.startsWith("MODE BASE TIME")) return "survey_in";
    if (mode.startsWith("MODE BASE")) return "base";
    if (mode.startsWith("MODE ROVER")) return "rover";
    return "unknown";
}

} // namespace UnicoreReadback
