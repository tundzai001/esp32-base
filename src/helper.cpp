#include "helper.h"
#include "functions/system/cmd_handler.h"
#include <string>

void SerialCommandProcessor::execute(String command)
{
    command.trim();
    if (command.isEmpty())
    {
        return;
    }

    Serial.println("[SERIAL COMMAND] " + command);
    std::vector<String> cmdWords = splitCommand(command);
    if (handleCommand(cmdWords) == cmd_action_t::CMD_ACTION_ESP_RESTART)
    {
        Serial.println("[SERIAL COMMAND] Yeu cau ESP32 khoi dong lai.");
        shutdownTcpTransportBeforeRestart();
        ESP.restart();
    }
}

void SerialCommandProcessor::processPending()
{
    while (Serial.available() > 0)
    {
        const auto character = static_cast<char>(Serial.read());

        if (character == '\r')
        {
            continue;
        }

        if (character == '\n')
        {
            execute(commandBuffer);
            commandBuffer = "";
            continue;
        }

        if (commandBuffer.length() < MAX_COMMAND_LENGTH)
        {
            commandBuffer += character;
        }
    }
}

void shutdownTcpTransportBeforeRestart()
{
    Serial.println("[SETUP] Dong cac ket noi TCP va GPRS truoc khi khoi dong lai...");
    mqtt.disconnect();
    activeNtripClient().stop();
    if (isGsmConnection() && modem.isGprsConnected())
    {
        modem.gprsDisconnect();
    }
    if (isWifiConnection())
    {
        WiFi.disconnect();
        delay(500);
    }
}

String formDeviceHealthString([[maybe_unused]] int32_t signalQualityDbm, const bool gnssDataOk,
                              [[maybe_unused]] const uint32_t *rtcmMessageCounts)
{
    // 1. Lấy các thông số hệ thống
    auto uptime_s = millis() / 1000;
    uint32_t freeHeap = ESP.getFreeHeap();

    const int32_t rssi = isWifiConnection() ? WiFi.RSSI() : signalQualityDbm;
    const String connected_via = isWifiConnection() ? "WiFi" : "GSM";

    auto mqttOk = isMqttConnected();
    auto ntripOk = isNtripConnected();
    // 2. Đóng gói thành JSON
    std::string healthPayload = "{";
    healthPayload += "\"uptime_s\":" + std::to_string(uptime_s);
    healthPayload += ",\"free_heap_bytes\":" + std::to_string(freeHeap);
    healthPayload += R"(,"connected_via":")" + std::string(connected_via.c_str()) + "\"";
    healthPayload += ",\"rssi_dbm\":" + std::to_string(rssi);
    healthPayload += ",\"mqtt_ok\":" + std::string(mqttOk ? "true" : "false");
    healthPayload += ",\"ntrip_ok\":" + std::string(ntripOk ? "true" : "false");
    healthPayload += ",\"gnss_data_ok\":" + std::string(gnssDataOk ? "true" : "false");
    healthPayload += ",\"rtcm_decoder\":false";
    healthPayload += ",\"rtcm_types\":{}";
    healthPayload += "}";
    // 3. Trả về payload để có thể log hoặc dùng cho mục đích khác nếu cần
    return String(healthPayload.c_str());
}
