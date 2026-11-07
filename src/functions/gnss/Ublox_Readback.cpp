#include "functions/gnss/Ublox_Readback.h"
#include "functions/gnss/Ubx_Reply.h"
#include "functions/ntrip/RTCM_Receiver.h"
#include "Prog_Config.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <array>

extern SemaphoreHandle_t rtcmBufferMutex;
extern SemaphoreHandle_t gnssSerialMutex;

namespace
{

    bool query(uint8_t cls, uint8_t id, UbxReply &reply, String &error)
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
        while (Serial1.available())
            bufferGnssQueryByte(static_cast<uint8_t>(Serial1.read()));
        const std::array<uint8_t, 8> poll = {
            0xB5, 0x62, cls, id, 0, 0, uint8_t(cls + id), uint8_t(4 * cls + 3 * id)};
        Serial.printf("[GNSS][READBACK][TX] UBX %02X/%02X; baud=%lu\n", cls, id,
                      static_cast<unsigned long>(Serial1.baudRate()));
        const auto written = Serial1.write(poll.data(), poll.size()) == poll.size();
        Serial1.flush();
        const uint32_t started = millis();
        auto matched = false;
        uint32_t receivedBytes = 0;
        while (written && millis() - started < 3000UL && !matched)
        {
            while (Serial1.available())
            {
                const auto value = static_cast<uint8_t>(Serial1.read());
                bufferGnssQueryByte(value);
                ++receivedBytes;
                if (reply.feed(value) && reply.frame[2] == cls && reply.frame[3] == id)
                {
                    matched = true;
                    break;
                }
            }
            if (!matched)
                vTaskDelay(pdMS_TO_TICKS(5));
        }
        xSemaphoreGive(gnssSerialMutex);
        xSemaphoreGive(rtcmBufferMutex);
        const char *result;
        if (!written)
            result = "WRITE_FAILED";
        else
            result = matched ? "OK" : "TIMEOUT";
        Serial.printf("[GNSS][READBACK][RX] UBX %02X/%02X; bytes=%lu; elapsed_ms=%lu; reply=%s\n",
                      cls, id, static_cast<unsigned long>(receivedBytes),
                      static_cast<unsigned long>(millis() - started), result);
        if (!matched)
            error = written ? "u-blox base readback timeout" : "u-blox query write failed";
        return matched;
    }

} // namespace

bool UbloxReadback::readBaseStatus(UbxBaseStatus &status, String &error)
{
    UbxReply mode;
    if (!query(0x06, 0x71, mode, error))
        return false;
    if (!status.tmode(mode.frame.data() + 6, mode.length - 8))
    {
        error = "Invalid CFG-TMODE3 response";
        return false;
    }
    if (status.mode != 1)
        return true;
    UbxReply survey;
    if (!query(0x01, 0x3B, survey, error))
        return false;
    if (!status.svin(survey.frame.data() + 6, survey.length - 8))
    {
        error = "Invalid NAV-SVIN response";
        return false;
    }
    return true;
}
