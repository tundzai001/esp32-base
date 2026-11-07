#include "functions/ntrip/RTCM_Receiver.h"

#include "HardwareSerial.h"
#include "Prog_Config.h"
#include "functions/backend/Backend_Agent.h"
#include <array>
#include <cstdio>

namespace
{

    constexpr size_t MAX_NMEA_LENGTH = 512;
    constexpr size_t MAX_RTCM_FRAME_LENGTH = 1029; // 3-byte header + 1023 + CRC

    String nmeaBuffer;
    String pendingRtcmData;
    std::array<uint8_t, MAX_RTCM_FRAME_LENGTH> rtcmFrame = {};
    size_t rtcmLength = 0;
    size_t rtcmExpectedLength = 0;
    std::array<uint8_t, 6> ubxHeader = {};
    size_t ubxHeaderLength = 0;
    size_t ubxBytesRemaining = 0;

    GnssTelemetrySnapshot telemetry;
    portMUX_TYPE telemetryMux = portMUX_INITIALIZER_UNLOCKED;

    uint32_t inputWindowStartedMs = 0;
    uint32_t inputWindowBytes = 0;
    uint32_t inputBytesPerSecond = 0;

    void noteInputBytes(size_t count)
    {
        portENTER_CRITICAL(&telemetryMux);
        telemetry.bytesRead += count;
        telemetry.hasData = true;
        telemetry.lastDataMs = millis();
        portEXIT_CRITICAL(&telemetryMux);
    }

    void noteRtcmBytes(size_t count)
    {
        const uint32_t now = millis();
        portENTER_CRITICAL(&telemetryMux);
        if (inputWindowStartedMs == 0)
            inputWindowStartedMs = now;
        inputWindowBytes += static_cast<uint32_t>(count);
        if (now - inputWindowStartedMs >= 1000UL)
        {
            const uint32_t elapsed = now - inputWindowStartedMs;
            inputBytesPerSecond = elapsed > 0
                                      ? static_cast<uint32_t>((static_cast<uint64_t>(inputWindowBytes) * 1000ULL) / elapsed)
                                      : inputWindowBytes;
            inputWindowStartedMs = now;
            inputWindowBytes = 0;
        }
        portEXIT_CRITICAL(&telemetryMux);
    }

    double nmeaCoordinate(const char *raw, const char *hemisphere)
    {
        if (!raw || !*raw)
            return 0.0;
        const auto value = atof(raw);
        const auto degrees = static_cast<int>(value / 100.0);
        const auto minutes = value - degrees * 100.0;
        auto coordinate = degrees + minutes / 60.0;
        if (hemisphere && (hemisphere[0] == 'S' || hemisphere[0] == 'W'))
        {
            coordinate = -coordinate;
        }
        return coordinate;
    }

    void setFixStatus(std::array<char, 16> &target, int quality)
    {
        const char *status;
        switch (quality)
        {
        case 0:
            status = "NO_FIX";
            break;
        case 1:
            status = "GPS_FIX";
            break;
        case 2:
            status = "DGPS";
            break;
        case 4:
            status = "RTK_FIXED";
            break;
        case 5:
            status = "RTK_FLOAT";
            break;
        case 7:
            status = "BASE_FIXED";
            break;
        default:
            status = "NO_FIX";
            break;
        }
        snprintf(target.data(), target.size(), "%s", status);
        target[target.size() - 1] = '\0';
    }

    void parseGgaFields(char *const *fields, size_t fieldCount, bool &updateFix,
                        int &fixQuality, bool &updatePosition, double &latitude,
                        double &longitude, double &altitude, float &hdop,
                        uint8_t &satellites)
    {
        updateFix = fieldCount > 6;
        fixQuality = updateFix ? atoi(fields[6]) : 0;
        if (fieldCount <= 9 || !fields[2][0] || !fields[3][0] || !fields[4][0] || !fields[5][0])
        {
            return;
        }
        latitude = nmeaCoordinate(fields[2], fields[3]);
        longitude = nmeaCoordinate(fields[4], fields[5]);
        satellites = static_cast<uint8_t>(constrain(atoi(fields[7]), 0, 255));
        hdop = static_cast<float>(atof(fields[8]));
        altitude = atof(fields[9]);
        updatePosition = true;
    }

    void parseGsvFields(char *const *fields, size_t fieldCount, bool &updateSnr,
                        float &snrSum, uint32_t &snrCount)
    {
        for (size_t i = 4; i + 3 < fieldCount; i += 4)
        {
            if (!fields[i + 3] || !*fields[i + 3])
                continue;
            const auto snr = atoi(fields[i + 3]);
            if (snr < 0 || snr > 99)
                continue;
            snrSum += static_cast<float>(snr);
            ++snrCount;
        }
        updateSnr = snrCount > 0;
    }

    void parseGstFields(char *const *fields, size_t fieldCount, bool &updateGst,
                        float &gstSemiMajor, float &gstSemiMinor, float &gstHacc)
    {
        if (fieldCount <= 7)
            return;
        gstSemiMajor = static_cast<float>(atof(fields[3]));
        gstSemiMinor = static_cast<float>(atof(fields[4]));
        const auto latitudeSigma = static_cast<float>(atof(fields[6]));
        const auto longitudeSigma = static_cast<float>(atof(fields[7]));
        gstHacc = sqrtf(latitudeSigma * latitudeSigma + longitudeSigma * longitudeSigma);
        updateGst = gstHacc > 0.0F;
    }

    void parseNmeaSentence(const String &rawSentence)
    {
        auto sentence = rawSentence;
        sentence.trim();
        if (sentence.length() < 7 || sentence[0] != '$')
            return;
        if (sentence.startsWith("$PUBX"))
        {
            backendAgent.setDetectedChipIfUnknown("Ublox");
        }

        std::array<char, MAX_NMEA_LENGTH> line = {};
        const auto copyLength = min(sentence.length(), line.size() - 1);
        memcpy(line.data(), sentence.c_str(), copyLength);
        line[copyLength] = '\0';

        std::array<char *, 32> fields = {};
        auto fieldCount = 0;
        char *cursor = line.data() + 1;
        while (fieldCount < (std::size(fields)))
        {
            fields[fieldCount++] = cursor;
            char *comma = strchr(cursor, ',');
            if (!comma)
                break;
            *comma = '\0';
            cursor = comma + 1;
        }
        if (fieldCount == 0)
            return;

        const auto typeLength = strnlen(fields[0], sizeof(fields[0]));
        if (typeLength < 3)
            return;
        const char *type = fields[0] + typeLength - 3;

        auto updateFix = false;
        auto fixQuality = 0;
        auto updatePosition = false;
        auto updateSnr = false;
        auto latitude = 0.0;
        auto longitude = 0.0;
        auto altitude = 0.0;
        auto hdop = 0.0F;
        uint8_t satellites = 0;
        auto snrSum = 0.0F;
        uint32_t snrCount = 0;
        auto updateGst = false;
        auto gstSemiMajor = 0.0F;
        auto gstSemiMinor = 0.0F;
        auto gstHacc = 0.0F;

        if (strcmp(type, "GGA") == 0)
        {
            parseGgaFields(fields.data(), fieldCount, updateFix, fixQuality, updatePosition,
                           latitude, longitude, altitude, hdop, satellites);
        }
        else if (strcmp(type, "GSA") == 0)
        {
            // GSA field 2 is only the 2D/3D solution dimension, not the RTK
            // quality reported by GGA. Keep the last GGA fix state authoritative
            // so a following GSA sentence cannot turn RTK_FIXED into a generic
            // 2D/3D status.
        }
        else if (strcmp(type, "GSV") == 0)
        {
            parseGsvFields(fields.data(), fieldCount, updateSnr, snrSum, snrCount);
        }
        else if (strcmp(type, "GST") == 0)
        {
            parseGstFields(fields.data(), fieldCount, updateGst, gstSemiMajor,
                           gstSemiMinor, gstHacc);
        }

        portENTER_CRITICAL(&telemetryMux);
        telemetry.hasData = true;
        telemetry.lastDataMs = millis();
        ++telemetry.nmeaSentences;
        if (strcmp(type, "GGA") == 0)
            ++telemetry.ggaCount;
        else if (strcmp(type, "GSA") == 0)
            ++telemetry.gsaCount;
        else if (strcmp(type, "GSV") == 0)
            ++telemetry.gsvCount;
        else if (strcmp(type, "GST") == 0)
            ++telemetry.gstCount;

        if (updateFix)
            setFixStatus(telemetry.fixStatus, fixQuality);
        if (strcmp(type, "GGA") == 0)
        {
            snprintf(telemetry.lastGga.data(), telemetry.lastGga.size(), "%s", sentence.c_str());
            telemetry.lastGga[telemetry.lastGga.size() - 1] = '\0';
            telemetry.lastGgaMs = millis();
        }
        if (updatePosition)
        {
            telemetry.latitude = latitude;
            telemetry.longitude = longitude;
            telemetry.altitude = altitude;
            telemetry.hdop = hdop;
            telemetry.satellites = satellites;
        }
        if (updateSnr)
        {
            const uint32_t oldSamples = telemetry.snrSamples;
            telemetry.snrSamples += snrCount;
            telemetry.latestGsvSamples = snrCount;
            telemetry.lastGsvMs = millis();
            telemetry.latestAverageSnr = static_cast<float>(snrSum) / static_cast<float>(snrCount);
            telemetry.averageSnr = oldSamples == 0
                                       ? static_cast<float>(snrSum) / static_cast<float>(snrCount)
                                       : ((telemetry.averageSnr * static_cast<float>(oldSamples)) + static_cast<float>(snrSum)) / static_cast<float>(telemetry.snrSamples);
        }
        if (updateGst)
        {
            telemetry.gstSemiMajor = gstSemiMajor;
            telemetry.gstSemiMinor = gstSemiMinor;
            telemetry.gstHacc = gstHacc;
            telemetry.lastGstMs = millis();
        }
        portEXIT_CRITICAL(&telemetryMux);

        // Publish only GGA/GSA/GSV/GST receiver health sentences to the backend.
        const auto backendSentence = strcmp(type, "GGA") == 0 || strcmp(type, "GSA") == 0 || strcmp(type, "GSV") == 0 || strcmp(type, "GST") == 0;
        if (backendSentence)
        {
            auto outbound = rawSentence;
            outbound.trim();
            outbound += "\r\n";
            backendAgent.enqueueNmea(reinterpret_cast<const uint8_t *>(outbound.c_str()), outbound.length());
        }
    }

    void resetRtcmParser()
    {
        rtcmLength = 0;
        rtcmExpectedLength = 0;
    }

    bool processRtcmByte(uint8_t value, String &rtcmOutput)
    {
        if (rtcmLength == 1 && (std::byte{value} & std::byte{0xFC}) != std::byte{})
        {
            resetRtcmParser();
            return false;
        }
        if (rtcmLength >= rtcmFrame.size())
        {
            resetRtcmParser();
            return true;
        }

        rtcmFrame[rtcmLength++] = value;
        if (rtcmLength == 3)
        {
            const uint16_t payloadLength =
                (static_cast<uint16_t>(rtcmFrame[1]) & 0x03U) * 256U + rtcmFrame[2];
            rtcmExpectedLength = static_cast<size_t>(payloadLength) + 6U;
            if (rtcmExpectedLength > rtcmFrame.size())
            {
                resetRtcmParser();
                return true;
            }
        }
        if (rtcmExpectedLength == 0 || rtcmLength != rtcmExpectedLength)
            return true;

        // Forward the complete RTCM frame without validating its CRC or decoding its type.
        rtcmOutput.concat(reinterpret_cast<const char *>(rtcmFrame.data()), rtcmLength);
        backendAgent.enqueueRtcm(rtcmFrame.data(), rtcmLength);
        noteRtcmBytes(rtcmLength);
        portENTER_CRITICAL(&telemetryMux);
        ++telemetry.rtcmFrames;
        telemetry.hasData = true;
        telemetry.lastDataMs = millis();
        portEXIT_CRITICAL(&telemetryMux);
        resetRtcmParser();
        return true;
    }

    bool processUbxHeaderByte(uint8_t value)
    {
        if (ubxHeaderLength == 1 && value != 0x62)
        {
            ubxHeaderLength = 0;
            return false;
        }

        ubxHeader[ubxHeaderLength++] = value;
        if (ubxHeaderLength != ubxHeader.size())
            return true;

        const auto payloadLength = size_t(ubxHeader[4]) | (size_t(ubxHeader[5]) << 8);
        ubxHeaderLength = 0;
        if (payloadLength <= 8192)
            ubxBytesRemaining = payloadLength + 2;
        return true;
    }

    void consumeByte(uint8_t value, String &rtcmOutput)
    {
        // UBX payload is binary and may contain '$', LF or 0xD3. Skip the whole
        // frame before looking for NMEA/RTCM, including across UART read chunks.
        if (ubxBytesRemaining > 0)
        {
            --ubxBytesRemaining;
            return;
        }
        if (ubxHeaderLength > 0 && processUbxHeaderByte(value))
            return;
        if (rtcmLength > 0 && processRtcmByte(value, rtcmOutput))
            return;

        if (nmeaBuffer.length() > 0)
        {
            if (value == '\n')
            {
                nmeaBuffer += static_cast<char>(value);
                parseNmeaSentence(nmeaBuffer);
                nmeaBuffer = "";
            }
            else if (value == '$')
            {
                // A malformed/incomplete line should not swallow the next valid
                // sentence when the GNSS stream recovers.
                nmeaBuffer = "$";
            }
            else if ((value < 0x20 && value != '\r') || value > 0x7E)
            {
                nmeaBuffer = "";
                consumeByte(value, rtcmOutput);
            }
            else if (nmeaBuffer.length() < MAX_NMEA_LENGTH - 1)
            {
                nmeaBuffer += static_cast<char>(value);
            }
            else
            {
                nmeaBuffer = "";
            }
            return;
        }

        if (value == 0xB5)
        {
            ubxHeader[0] = value;
            ubxHeaderLength = 1;
        }
        else if (value == 0xD3)
        {
            rtcmFrame[0] = value;
            rtcmLength = 1;
            rtcmExpectedLength = 0;
        }
        else if (value == '$')
        {
            nmeaBuffer = "$";
        }
    }

} // namespace

void bufferGnssQueryByte(uint8_t value)
{
    noteInputBytes(1);
    consumeByte(value, pendingRtcmData);
}

String receiveRtcmFromGnss()
{
    String rtcmData;
    rtcmData.reserve(1024);
    std::array<uint8_t, 128> buffer = {};

    if (gnssSerialMutex == nullptr ||
        !xSemaphoreTake(gnssSerialMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)))
    {
        return rtcmData;
    }

    if (!pendingRtcmData.isEmpty())
    {
        rtcmData = pendingRtcmData;
        pendingRtcmData = "";
    }

    while (Serial1.available() > 0)
    {
        const auto bytesAvailable = static_cast<size_t>(Serial1.available());
        const auto bytesToRead = min(bytesAvailable, buffer.size());
        auto stopReading = bytesToRead == 0;

        size_t bytesRead = 0;
        if (!stopReading)
        {
            bytesRead = Serial1.readBytes(
                reinterpret_cast<char *>(buffer.data()), bytesToRead);
            stopReading = bytesRead == 0;
        }
        if (stopReading)
            break;
        noteInputBytes(bytesRead);
        for (size_t i = 0; i < bytesRead; ++i)
        {
            consumeByte(buffer[i], rtcmData);
        }
    }
    xSemaphoreGive(gnssSerialMutex);

    // Count forwarded RTCM bytes for the periodic UART diagnostic log.
    static uint32_t lastLogMs = 0;
    static uint32_t receivedBytes = 0;
    receivedBytes += rtcmData.length();
    if (millis() - lastLogMs >= 5000UL)
    {
        Serial.printf("[RTCM] Framed UART bytes in last interval (CRC chua kiem tra): %lu\n",
                      static_cast<unsigned long>(receivedBytes));
        const auto snapshot = getGnssTelemetrySnapshot();
        Serial.printf("[GNSS UART] GGA total=%lu; last_age_ms=%lu; RX bytes=%llu\n",
                      static_cast<unsigned long>(snapshot.ggaCount),
                      static_cast<unsigned long>(snapshot.lastGgaMs == 0 ? UINT32_MAX : millis() - snapshot.lastGgaMs),
                      static_cast<unsigned long long>(snapshot.bytesRead));
        receivedBytes = 0;
        lastLogMs = millis();
    }
    return rtcmData;
}

RtcmMessageCounts consumeRtcmMessageCounts()
{
    // Return zero per-type counters; this parser does not classify RTCM messages.
    return RtcmMessageCounts{};
}

GnssTelemetrySnapshot getGnssTelemetrySnapshot()
{
    GnssTelemetrySnapshot copy;
    portENTER_CRITICAL(&telemetryMux);
    copy = telemetry;
    portEXIT_CRITICAL(&telemetryMux);
    if (copy.lastGgaMs == 0 || millis() - copy.lastGgaMs > 15000UL)
    {
        copy.fixStatus = {'N', 'O', '_', 'F', 'I', 'X'};
    }
    return copy;
}

uint32_t getRtcmInputBps()
{
    const uint32_t now = millis();
    portENTER_CRITICAL(&telemetryMux);
    if (inputWindowStartedMs > 0 && now - inputWindowStartedMs >= 1000UL)
    {
        const uint32_t elapsed = now - inputWindowStartedMs;
        inputBytesPerSecond = elapsed > 0
                                  ? static_cast<uint32_t>((static_cast<uint64_t>(inputWindowBytes) * 1000ULL) / elapsed)
                                  : inputWindowBytes;
        inputWindowStartedMs = now;
        inputWindowBytes = 0;
    }
    const uint32_t result = inputBytesPerSecond;
    portEXIT_CRITICAL(&telemetryMux);
    return result;
}
