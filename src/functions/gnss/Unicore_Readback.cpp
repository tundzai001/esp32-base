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

namespace UnicoreReadback
{
    namespace
    {
        bool appendUnicoreCandidate(char value, const char *prefix, String &candidate,
                                    String &response)
        {
            if (value == prefix[0])
            {
                candidate = String(value);
                return false;
            }
            if (value == '\r' || value == '\n')
            {
                if (candidate.startsWith(prefix))
                    response = candidate;
                candidate = "";
                return !response.isEmpty();
            }
            if (!candidate.isEmpty() && candidate.length() < 383)
            {
                candidate += value;
            }
            else if (candidate.length() >= 383)
            {
                candidate = "";
            }
            return false;
        }

        bool processUnicoreQueryByte(uint8_t byte, const char *prefix, UnicoreReply &reply,
                                     String &candidate, String &response, String &error)
        {
            const auto value = static_cast<char>(byte);
            bufferGnssQueryByte(byte);
            const auto acknowledged = reply.feed(byte);
            if (acknowledged && reply.state == UnicoreReply::State::Error)
            {
                Serial.printf("[GNSS][READBACK][ACK] %s\n", reply.line.data());
                error = String("Unicore rejected query: ") + reply.line.data();
                return false;
            }
            if (acknowledged)
                Serial.printf("[GNSS][READBACK][ACK] %s\n", reply.line.data());
            appendUnicoreCandidate(value, prefix, candidate, response);
            return true;
        }

        void collectUnicoreResponse(const char *prefix, UnicoreReply &reply,
                                    String &response, String &error, uint32_t &receivedBytes)
        {
            String candidate;
            candidate.reserve(160);
            const uint32_t started = millis();
            while (millis() - started < 3000UL && response.isEmpty() && error.isEmpty())
            {
                while (Serial1.available() > 0)
                {
                    const auto value = static_cast<uint8_t>(Serial1.read());
                    ++receivedBytes;
                    if (!processUnicoreQueryByte(value, prefix, reply, candidate, response, error))
                        break;
                }
                if (response.isEmpty() && error.isEmpty())
                    vTaskDelay(pdMS_TO_TICKS(5));
            }
        }

        bool queryUnicoreLine(const char *query, const char *prefix, String &response, String &error)
        {
            if (!rtcmBufferMutex || xSemaphoreTake(rtcmBufferMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) != pdTRUE)
            {
                error = "GNSS command port is busy";
                return false;
            }
            if (!gnssSerialMutex || xSemaphoreTake(gnssSerialMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) != pdTRUE)
            {
                xSemaphoreGive(rtcmBufferMutex);
                error = "GNSS reader is busy";
                return false;
            }

            while (Serial1.available() > 0)
                bufferGnssQueryByte(static_cast<uint8_t>(Serial1.read()));
            String command(query);
            command += "\r\n";
            UnicoreReply reply(reinterpret_cast<const uint8_t *>(command.c_str()), command.length());
            Serial.printf("[GNSS][READBACK][TX] %s; baud=%lu\n", query,
                          static_cast<unsigned long>(Serial1.baudRate()));
            // Finish any partial input line left on the mixed binary/ASCII command port.
            const auto written = Serial1.write(reinterpret_cast<const uint8_t *>("\r\n"), 2) == 2 && Serial1.write(
                                                                                                         reinterpret_cast<const uint8_t *>(command.c_str()), command.length()) == command.length();
            Serial1.flush();
            uint32_t receivedBytes = 0;
            const uint32_t started = millis();
            if (written)
                collectUnicoreResponse(prefix, reply, response, error, receivedBytes);
            xSemaphoreGive(gnssSerialMutex);
            xSemaphoreGive(rtcmBufferMutex);

            Serial.printf("[GNSS][READBACK][RX] query=%s; bytes=%lu; elapsed_ms=%lu; reply=%u\n",
                          query, static_cast<unsigned long>(receivedBytes),
                          static_cast<unsigned long>(millis() - started), static_cast<unsigned int>(reply.state));

            if (!error.isEmpty())
                return false;
            if (!written)
            {
                error = String("Unicore ") + query + " query write failed";
                return false;
            }
            if (response.isEmpty())
            {
                error = String("Unicore ") + query + " readback timeout";
                return false;
            }
            return true;
        }

    } // namespace

    bool readMode(String &mode, String &error)
    {
        String response;
        if (!queryUnicoreLine("MODE", "#MODE", response, error))
        {
            Serial.printf("[GNSS][READBACK][ERROR] %s\n", error.c_str());
            return false;
        }
        Serial.printf("[GNSS][READBACK] %s\n", response.c_str());
        if (!validUnicoreLogChecksum(response.c_str(), response.length()))
        {
            error = "Unicore MODE checksum mismatch";
            Serial.printf("[GNSS][READBACK][ERROR] %s\n", error.c_str());
            return false;
        }
        const auto separator = response.indexOf(';');
        if (separator < 0)
        {
            error = "Unicore MODE response is malformed";
            return false;
        }
        mode = response.substring(separator + 1);
        const auto checksum = mode.indexOf('*');
        if (checksum >= 0)
            mode.remove(checksum);
        mode.trim();
        if (mode.endsWith(","))
            mode.remove(mode.length() - 1);
        mode.toUpperCase();
        if (!mode.startsWith("MODE "))
        {
            error = "Unicore MODE response has no operating mode";
            mode = "";
            return false;
        }
        return true;
    }

    bool readBestPosition(JsonObject position, String &error)
    {
        String response;
        if (!queryUnicoreLine("BESTNAVA", "#BESTNAVA,", response, error))
            return false;
        if (!validUnicoreLogChecksum(response.c_str(), response.length()))
        {
            error = "Unicore BESTNAV checksum mismatch";
            return false;
        }
        position["message"] = "BESTNAV";
        position["raw"] = response;
        const auto separator = response.indexOf(';');
        if (separator < 0)
        {
            error = "Unicore BESTNAV response is malformed";
            return false;
        }
        auto payload = response.substring(separator + 1);
        const auto checksum = payload.indexOf('*');
        if (checksum >= 0)
            payload.remove(checksum);
        std::array<String, 5> fields;
        auto start = 0;
        for (size_t index = 0; index < 5; ++index)
        {
            const auto comma = payload.indexOf(',', start);
            if (index < 4 && comma < 0)
            {
                error = "Unicore BESTNAV response is incomplete";
                return false;
            }
            const auto end = comma < 0 ? payload.length() : comma;
            fields[index] = payload.substring(start, end);
            start = end + 1;
        }
        for (String &field : fields)
            field.trim();
        position["solution_status"] = fields[0];
        position["position_type"] = fields[1];
        if (!fields[0].equalsIgnoreCase("SOL_COMPUTED") || fields[2].isEmpty() || fields[3].isEmpty() || fields[4].isEmpty())
        {
            error = String("Unicore BESTNAV has no computed position: ") + fields[0];
            return false;
        }
        char *end = nullptr;
        const auto latitude = std::strtod(fields[2].c_str(), &end);
        if (end == fields[2].c_str() || *end != '\0')
        {
            error = "Unicore BESTNAV latitude is invalid";
            return false;
        }
        const auto longitude = std::strtod(fields[3].c_str(), &end);
        if (end == fields[3].c_str() || *end != '\0')
        {
            error = "Unicore BESTNAV longitude is invalid";
            return false;
        }
        const auto height = std::strtod(fields[4].c_str(), &end);
        if (end == fields[4].c_str() || *end != '\0')
        {
            error = "Unicore BESTNAV height is invalid";
            return false;
        }
        if (!std::isfinite(latitude) || !std::isfinite(longitude) || !std::isfinite(height) || latitude < -90.0 || latitude > 90.0 || longitude < -180.0 || longitude > 180.0)
        {
            error = "Unicore BESTNAV coordinates are invalid";
            return false;
        }
        position["state"] = "ok";
        position["latitude"] = latitude;
        position["longitude"] = longitude;
        position["height_msl"] = height;
        return true;
    }

    const char *modeKind(const String &mode)
    {
        if (mode.startsWith("MODE BASE TIME"))
            return "survey_in";
        if (mode.startsWith("MODE BASE"))
            return "base";
        if (mode.startsWith("MODE ROVER"))
            return "rover";
        return "unknown";
    }

} // namespace UnicoreReadback
