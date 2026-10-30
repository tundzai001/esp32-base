#include "functions/cmd_handler.h"
#include <vector>
#include <Preferences.h>
#include "Prog_Config.h"
#include "functions/gnss_cmd_builder.h"
#include "functions/Backend_Agent.h"
#include <cstring>

extern SemaphoreHandle_t rtcmBufferMutex;

extern Preferences prefs;

namespace cmd_helper {

    void printGnssResponse(const uint8_t *data, size_t length)
    {
        bool printable = length > 0;
        for (size_t i = 0; i < length; ++i) {
            if (data[i] < 0x20 && data[i] != '\r' && data[i] != '\n' && data[i] != '\t') {
                printable = false;
                break;
            }
        }

        if (printable) {
            String response;
            response.reserve(length);
            for (size_t i = 0; i < length; ++i) {
                response += static_cast<char>(data[i]);
            }
            Serial.println("[GNSS RESPONSE][ASCII] " + response);
            return;
        }

        Serial.print("[GNSS RESPONSE][HEX] ");
        for (size_t i = 0; i < length; ++i) {
            if (data[i] < 0x10) {
                Serial.print('0');
            }
            Serial.print(data[i], HEX);
            if (i + 1 < length) {
                Serial.print(' ');
            }
        }
        Serial.println();
    }

    void readGnssResponse()
    {
        constexpr uint32_t RESPONSE_TIMEOUT_MS = 500;
        constexpr uint32_t RESPONSE_IDLE_MS = 80;
        constexpr size_t RESPONSE_BUFFER_SIZE = 512;
        uint8_t response[RESPONSE_BUFFER_SIZE] = {};
        size_t length = 0;
        uint32_t lastByteAt = millis();
        const uint32_t startedAt = millis();

        while (millis() - startedAt < RESPONSE_TIMEOUT_MS) {
            while (Serial1.available() > 0) {
                const int value = Serial1.read();
                if (length < RESPONSE_BUFFER_SIZE) {
                    response[length++] = static_cast<uint8_t>(value);
                }
                lastByteAt = millis();
            }

            if (length > 0 && millis() - lastByteAt >= RESPONSE_IDLE_MS) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(2));
        }

        if (length == 0) {
            Serial.println("[GNSS RESPONSE] Khong co phan hoi trong thoi gian cho.");
        } else {
            printGnssResponse(response, length);
        }
    }

    struct PreferenceMapping {
        const char *command;
        const char *preferenceKey;
    };

    inline bool hasExactArgumentCount(const std::vector<String> &cmdWords, size_t expectedCount,
                            const char *commandName)
    {
        if (cmdWords.size() == expectedCount) {
            return true;
        }

        Serial.println("[MQTT COMMAND DOWNLINK] Error: Lenh " + String(commandName) +
                    (cmdWords.size() < expectedCount ? " khong day du." : " qua dai!"));
        return false;
    }

    inline cmd_action_t saveStringPreference(const char *key, const String &value)
    {
        prefs.begin("myPrefs", false);
        prefs.putString(key, value);
        prefs.end();
        return CMD_ACTION_NONE;
    }

    inline cmd_action_t saveUShortPreference(const char *key, uint16_t value)
    {
        prefs.begin("myPrefs", false);
        prefs.putUShort(key, value);
        prefs.end();
        return CMD_ACTION_NONE;
    }

    inline cmd_action_t saveIntPreference(const char *key, int value)
    {
        prefs.begin("myPrefs", false);
        prefs.putInt(key, value);
        prefs.end();
        return CMD_ACTION_NONE;
    }

    inline const char *findPreferenceKey(const String &command,
                                const PreferenceMapping *mappings, size_t mappingCount)
    {
        for (size_t i = 0; i < mappingCount; ++i) {
            if (command == mappings[i].command) {
                return mappings[i].preferenceKey;
            }
        }
        return nullptr;
    }

    inline void sendGnssCommands(const GnssCmdBuilder::CommandList &commands)
    {
        const GnssCmdBuilder::Command commandBytes = GnssCmdBuilder::commandListToBytes(commands);
        if (gnssSerialMutex == nullptr ||
            !xSemaphoreTake(gnssSerialMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS))) {
            Serial.println("[GNSS COMMAND][ERROR] Khong lay duoc mutex Serial1.");
            return;
        }
        if (!commandBytes.empty()) {
            Serial1.write(commandBytes.data(), commandBytes.size());
        }
        Serial1.flush();
        readGnssResponse();
        xSemaphoreGive(gnssSerialMutex);
    }

} // namespace cmd_helper

std::vector<String> splitCommand(const String &command) {
    std::vector<String> cmdWords;

    int start = 0;
    const int commandLength = command.length();

    while (start <= commandLength) {
        const int end = command.indexOf(' ', start);
        String word = end == -1 ? command.substring(start) : command.substring(start, end);
        word.trim();

        if (!word.isEmpty()) {
            cmdWords.emplace_back(word);
        }

        if (end == -1) {
            break;
        }

        start = end + 1;
    }

    return cmdWords;
}

cmd_action_t handleGnssBaseCommand(const std::vector<String> &cmdWords) {
    GnssCmdBuilder::GnssOptions options;
    options.ports = {"UART1"};
    options.outputMode = GnssCmdBuilder::OutputMode::RtcmOnly;
    GnssCmdBuilder::CommandList commands;
    prefs.begin("myPrefs", true);
    const String configuredSensorType = prefs.isKey("GNSS_MDL_TYPE")
        ? prefs.getString("GNSS_MDL_TYPE")
        : "UBLOX";
    const GnssCmdBuilder::GnssSensorType sensorType =
        GnssCmdBuilder::sensorTypeFromString(configuredSensorType);
    if (sensorType == GnssCmdBuilder::GnssSensorType::Invalid) {
        prefs.end();
        Serial.println("[MQTT COMMAND DOWNLINK] LOI - Chip GNSS dang o trang thai INVALID.");
        return CMD_ACTION_NONE;
    }
    prefs.end();

    if (cmdWords.empty()) {
        Serial.println("[MQTT COMMAND DOWNLINK] Error: Lenh GNSS BASE khong day du.");
        return CMD_ACTION_NONE;
    }

    if (cmdWords[0] == "SURVEY_IN") {
        Serial.println("[MQTT COMMAND DOWNLINK] Lenh yeu cau cau hinh GNSS BASE SURVEY_IN");
        if (cmdWords.size() != 3 && cmdWords.size() != 4) {
            Serial.println("[MQTT COMMAND DOWNLINK] LOI - GNSS BASE SURVEY_IN can 2 hoac 3 tham so.");
            return CMD_ACTION_NONE;
        }
        if (cmdWords.size() == 4) {
            String outputMode = cmdWords[3];
            outputMode.toUpperCase();
            if (outputMode != "W_DIAG") {
                Serial.println("[MQTT COMMAND DOWNLINK] LOI - Tham so thu ba phai la W_DIAG.");
                return CMD_ACTION_NONE;
            }
            options.outputMode = GnssCmdBuilder::OutputMode::DiagnosticsRtcm;
        }
        uint32_t duration = cmdWords[1].toInt();
        float accuracy = cmdWords[2].toFloat();
        
        commands = GnssCmdBuilder::buildBaseSurveyInCommand(duration, accuracy, sensorType, options);
        cmd_helper::sendGnssCommands(commands);
        Serial.println("[MQTT COMMAND DOWNLINK] Da gui lenh cau hinh GNSS BASE SURVEY_IN.");

        return CMD_ACTION_NONE;
    } else if (cmdWords[0] == "FIXED") {
        Serial.println("[MQTT COMMAND DOWNLINK] Lenh yeu cau cau hinh GNSS FIXED LLA");
        if (!cmd_helper::hasExactArgumentCount(cmdWords, 5, "GNSS BASE FIXED")) {
            return CMD_ACTION_NONE;
        }
        double lat = cmdWords[1].toDouble();
        double lon = cmdWords[2].toDouble();
        double alt = cmdWords[3].toDouble();
        float accuracy = cmdWords[4].toFloat();

        commands = GnssCmdBuilder::buildBaseFixedLlaCommand(lat, lon, alt, accuracy, sensorType, options);
        cmd_helper::sendGnssCommands(commands);
        Serial.println("[MQTT COMMAND DOWNLINK] Da gui lenh cau hinh GNSS BASE FIXED LLA.");

        return CMD_ACTION_NONE;
    } else if (cmdWords.size() == 3) {
        String message = cmdWords[0];
        message.toUpperCase();
        String action = cmdWords[2];
        action.toUpperCase();
        if (action != "ON" && action != "OFF") {
            Serial.println("[MQTT COMMAND DOWNLINK] LOI - Trang thai RTCM phai la ON hoac OFF.");
            return CMD_ACTION_NONE;
        }
        const bool enabled = action == "ON";
        Serial.println("[MQTT COMMAND DOWNLINK] Lenh yeu cau " + action + " RTCM " + cmdWords[0] +
                       " tren cong " + cmdWords[1]);
        commands = message == "ALL"
            ? GnssCmdBuilder::buildBaseAllRtcmOutputCommand(cmdWords[1], enabled, sensorType)
            : GnssCmdBuilder::buildBaseRtcmOutputCommand(cmdWords[0], cmdWords[1], enabled, sensorType);
        if (commands.empty()) {
            Serial.println("[MQTT COMMAND DOWNLINK] LOI - RTCM message hoac cong khong hop le.");
            return CMD_ACTION_NONE;
        }
        cmd_helper::sendGnssCommands(commands);
        Serial.println("[MQTT COMMAND DOWNLINK] Da gui lenh RTCM.");
        return CMD_ACTION_NONE;
    } else {
        Serial.println("[MQTT COMMAND DOWNLINK] LOI - Lenh khong kha dung: " + cmdWords[0]);
        return CMD_ACTION_NONE;
    }
}

cmd_action_t handleGnssSetCommand(const std::vector<String> &cmdWords) {
    if (!cmd_helper::hasExactArgumentCount(cmdWords, 3, "GNSS SET")) {
        return CMD_ACTION_NONE;
    }

    if (cmdWords[1] != "CHIP") {
        Serial.println("[MQTT COMMAND DOWNLINK] LOI - Lenh GNSS SET khong kha dung.");
        return CMD_ACTION_NONE;
    }

    String sensorType = cmdWords[2];
    sensorType.toUpperCase();
    if (sensorType != "UBLOX" && sensorType != "UNICORE") {
        Serial.println("[MQTT COMMAND DOWNLINK] LOI - Chip GNSS phai la UBLOX hoac UNICORE.");
        return CMD_ACTION_NONE;
    }

    cmd_helper::saveStringPreference("GNSS_MDL_TYPE", sensorType);
    Serial.println("[MQTT COMMAND DOWNLINK] Da chuyen chip GNSS sang " + sensorType +
                   ", dang khoi dong lai ESP32...");
    return CMD_ACTION_ESP_RESTART;
}

cmd_action_t handleEspCommand(const std::vector<String> &cmdWords) {
    if (cmdWords.empty()) {
        Serial.println("[MQTT COMMAND DOWNLINK] Error: Lenh ESP khong day du.");
        return CMD_ACTION_NONE;
    }

    if (cmdWords[0] == "RESTART") {
        Serial.println("[MQTT COMMAND DOWNLINK] Lenh yeu cau khoi dong lai ESP32");
        return CMD_ACTION_ESP_RESTART;
    }

    if (cmdWords[0] == "SET" && cmd_helper::hasExactArgumentCount(cmdWords, 3, "ESP SET")) {
        if (cmdWords[1] == "CONNECTION" && cmdWords[2] == "4G") {
            Serial.println("[MQTT COMMAND DOWNLINK] Lenh yeu cau cau hinh 4G");
            cmd_helper::saveStringPreference("CONNECTION_TYPE", "4G");
            return CMD_ACTION_ESP_RESTART;
        }
        if (cmdWords[1] == "CONNECTION" && cmdWords[2] == "WIFI") {
            Serial.println("[MQTT COMMAND DOWNLINK] Lenh yeu cau cau hinh WIFI");
            cmd_helper::saveStringPreference("CONNECTION_TYPE", "WIFI");
            return CMD_ACTION_ESP_RESTART;
        }
    }

    if (cmdWords[0] == "SET" && cmd_helper::hasExactArgumentCount(cmdWords, 4, "ESP SET")) {
        if (cmdWords[1] == "WIFI" && cmdWords[2] == "SSID") {
            Serial.println("[MQTT COMMAND DOWNLINK] Lenh yeu cau cau hinh WIFI SSID");
            return cmd_helper::saveStringPreference("WIFI_SSID", cmdWords[3]);
        }
        if (cmdWords[1] == "WIFI" && cmdWords[2] == "PASS") {
            Serial.println("[MQTT COMMAND DOWNLINK] Lenh yeu cau cau hinh WIFI PASS");
            return cmd_helper::saveStringPreference("WIFI_PASS", cmdWords[3]);
        }
        if (cmdWords[1] == "4G" && cmdWords[2] == "APN") {
            Serial.println("[MQTT COMMAND DOWNLINK] Lenh yeu cau cau hinh 4G APN");
            return cmd_helper::saveStringPreference("APN", cmdWords[3]);
        }
        if (cmdWords[1] == "4G" && cmdWords[2] == "USER") {
            Serial.println("[MQTT COMMAND DOWNLINK] Lenh yeu cau cau hinh 4G USER");
            return cmd_helper::saveStringPreference("GPRS_USER", cmdWords[3]);
        }
        if (cmdWords[1] == "4G" && cmdWords[2] == "PASS") {
            Serial.println("[MQTT COMMAND DOWNLINK] Lenh yeu cau cau hinh 4G PASS");
            return cmd_helper::saveStringPreference("GPRS_PASS", cmdWords[3]);
        }
        if (cmdWords[1] == "BACKEND" && cmdWords[2] == "HOST") {
            return cmd_helper::saveStringPreference("BACKEND_HOST", cmdWords[3]);
        }
        if (cmdWords[1] == "BACKEND" && cmdWords[2] == "PORT") {
            const long port = cmdWords[3].toInt();
            if (port >= 1 && port <= 65535) {
                return cmd_helper::saveUShortPreference("BACKEND_PORT", static_cast<uint16_t>(port));
            }
            Serial.println("[BACKEND] Port must be 1..65535.");
            return CMD_ACTION_NONE;
        }
    }
    
    Serial.println("[MQTT COMMAND DOWNLINK] This command is not yet implemented.");
    return CMD_ACTION_NONE;
}

cmd_action_t handleMqttCommand(const std::vector<String> &cmdWords) {
    if (cmd_helper::hasExactArgumentCount(cmdWords, 3, "MQTT" ) && cmdWords[0] == "SET") {
        static constexpr cmd_helper::PreferenceMapping stringSettings[] = {
            {"SERVER", "MQTT_SERVER"}, {"USER", "MQTT_USER"},
            {"PASS", "MQTT_PASS"}, {"PUBTPCHEALTH", "TPC_HEALTH"},
            {"SUBTPCCMD", "TPC_SUB_CMD"},
        };
        const char *key = cmd_helper::findPreferenceKey(cmdWords[1], stringSettings,
                                            sizeof(stringSettings) / sizeof(stringSettings[0]));
        if (key != nullptr) {
            return cmd_helper::saveStringPreference(key, cmdWords[2]);
        }
        if (cmdWords[1] == "PORT") {
            return cmd_helper::saveUShortPreference("MQTT_PORT", static_cast<uint16_t>(cmdWords[2].toInt()));
        }
    }
    Serial.println("[MQTT COMMAND DOWNLINK] Lenh sai hoac khong kha dung.");
    return CMD_ACTION_NONE;
}

cmd_action_t handleNtripCommand(const std::vector<String> &cmdWords) {
    if (cmd_helper::hasExactArgumentCount(cmdWords, 3, "NTRIP") && cmdWords[0] == "SET") {
        static constexpr cmd_helper::PreferenceMapping stringSettings[] = {
            {"CSTRADDR", "NTRIP_SERVER"}, {"MNTPNT", "NTRIP_MPT"},
            {"CSTRAUTH", "NT_AUTH_BS"},
        };
        const char *key = cmd_helper::findPreferenceKey(cmdWords[1], stringSettings,
                                            sizeof(stringSettings) / sizeof(stringSettings[0]));
        if (key != nullptr) {
            return cmd_helper::saveStringPreference(key, cmdWords[2]);
        }
        if (cmdWords[1] == "CSTRPORT") {
            return cmd_helper::saveUShortPreference("NTRIP_PORT", static_cast<uint16_t>(cmdWords[2].toInt()));
        }
    }
    Serial.println("[MQTT COMMAND DOWNLINK] Lenh sai hoac khong kha dung.");
    return CMD_ACTION_NONE;
}

cmd_action_t handleCommand(std::vector<String> &cmdWords) {
    if (cmdWords.size() < 2 || cmdWords[0] != "ATG") {
        Serial.println("[MQTT COMMAND DOWNLINK] Error: Empty or invalid command format. Command must start with 'ATG'.");
        return CMD_ACTION_NONE;
    }
    if (cmdWords[1] == "GNSS") {
        if (cmdWords.size() < 3) {
            Serial.println("[MQTT COMMAND DOWNLINK] Error: Incomplete GNSS command.");
            return CMD_ACTION_NONE;
        }
        if (cmdWords.size() == 3 && cmdWords[2] == "ROVER") {
            if (backendAgent.workflowActive()) {
                Serial.println("[GNSS ROVER][ERROR] Dung workflow hien tai truoc khi thu lai");
            } else {
                const bool confirmed = backendAgent.applyRoverMode();
                Serial.println(confirmed ? "[GNSS ROVER] Rover va GGA da xac nhan"
                                         : "[GNSS ROVER][ERROR] Chua xac nhan Rover/GGA");
            }
            return CMD_ACTION_NONE;
        }
        if (cmdWords[2] == "BASE") {
            cmdWords.erase(cmdWords.begin(), cmdWords.begin() + 3);
            return handleGnssBaseCommand(cmdWords);
        }
        if (cmdWords[2] == "SET") {
            cmdWords.erase(cmdWords.begin(), cmdWords.begin() + 2);
            return handleGnssSetCommand(cmdWords);
        }
        Serial.println("[MQTT COMMAND DOWNLINK] Error: Unknown GNSS command: " + cmdWords[2]);
        return CMD_ACTION_NONE;
    }    
    if (cmdWords[1] == "ESP") {
        cmdWords.erase(cmdWords.begin(), cmdWords.begin() + 2);
        return handleEspCommand(cmdWords);
    }
    if (cmdWords[1] == "MQTT") {
        cmdWords.erase(cmdWords.begin(), cmdWords.begin() + 2);
        return handleMqttCommand(cmdWords);
    }
    if (cmdWords[1] == "NTRIP") {
        cmdWords.erase(cmdWords.begin(), cmdWords.begin() + 2);
        return handleNtripCommand(cmdWords);
    }
    if (cmdWords[1] == "CONFIG") {
        if (cmdWords.size() >= 3 && cmdWords[2] == "RESET") {
            Serial.println("[MQTT COMMAND DOWNLINK] Lenh yeu cau reset ve cau hinh mac dinh");
            prefs.begin("myPrefs", false);
            prefs.putBool("NEED_RESET", true);
            prefs.end();
            return CMD_ACTION_ESP_RESTART;
        }
    }
    Serial.println("[MQTT COMMAND DOWNLINK] Unknown command: " + cmdWords[1]);
    return CMD_ACTION_NONE;
}
