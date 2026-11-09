#include "functions/backend/Backend_Agent.h"
#include "functions/backend/Backend_Agent_Internal.h"
#include "functions/gnss/Unicore_Readback.h"
#include "functions/gnss/Ublox_Readback.h"
#include "functions/gnss/Ubx_Config_Verification.h"
#include "functions/gnss/Unicore_Config_Verification.h"
#include <cstdio>
#include <cstring>
#include "Prog_Config.h"
#include "functions/ntrip/RTCM_Receiver.h"
#include "functions/gnss/Ubx_Reply.h"
#include "functions/gnss/Unicore_Reply.h"

using namespace BackendAgentInternal;

extern SemaphoreHandle_t rtcmBufferMutex;

namespace {
void describeTmode(JsonObject object, const uint8_t *raw) {
    object["mode"] = raw[2] == 2 ? "Fixed" : raw[2] == 1 ? "Survey-In" : "Rover";
    object["coordinate_type"] = raw[3] & 1 ? "LLA" : "ECEF";
    object["accuracy_m"] = UbxBaseStatus::u32(raw + 20) * 0.0001;
    UbxBaseStatus status;
    if (status.tmode(raw, 40) && status.hasPosition) {
        JsonObject position = object.createNestedObject("self_position");
        char value[32];
        snprintf(value, sizeof(value), "%.11f", status.latitude); position["lat"] = value;
        snprintf(value, sizeof(value), "%.11f", status.longitude); position["lon"] = value;
        snprintf(value, sizeof(value), "%.4f", status.height); position["alt"] = value;
    }
}
}

bool BackendAgent::verifyUnicoreConfig(JsonObjectConst payload, const String &commandId, String &error) {
    gnssVerification_ = static_cast<const char *>(nullptr);
    nvsInventory_ = static_cast<const char *>(nullptr);
    const JsonArrayConst expectedLogs = payload["logs"].as<JsonArrayConst>();
    const String reference = payload["altitude_reference"] | "LEGACY";
    double wanted[3];
    size_t coordinate = 0;
    for (const char *key : {"lat", "lon", "alt"}) {
        const JsonVariantConst input = payload["coords"][key];
        double value = NAN;
        if (input.is<const char *>()) {
            const char *text = input.as<const char *>();
            char *end = nullptr;
            value = std::strtod(text, &end);
            if (end == text || *end) value = NAN;
        } else if (!input.is<bool>() && input.is<double>()) {
            value = input.as<double>();
        }
        if (!std::isfinite(value)) {
            error = "Invalid Unicore verification coordinates"; return false;
        }
        wanted[coordinate++] = value;
    }
    if (std::abs(wanted[0]) > 90 || std::abs(wanted[1]) > 180 || wanted[2] < -12000 || wanted[2] > 100000
        || (reference != "LEGACY" && reference != "ELLIPSOID")
        || expectedLogs.isNull() || !expectedLogs.size() || expectedLogs.size() > 64) {
        error = "Invalid Unicore verification expectation"; return false;
    }
    const String requestedPort = payload["port"] | "CURRENT";
    uint8_t selectedPort = 0;
    if (requestedPort != "CURRENT") {
        if (requestedPort != "COM1" && requestedPort != "COM2" && requestedPort != "COM3") {
            error = "Invalid Unicore comparison port"; return false;
        }
        selectedPort = requestedPort[3] - '0';
    }
    std::vector<String> names;
    names.reserve(expectedLogs.size());
    for (JsonVariantConst item : expectedLogs) {
        const char *name = item.as<const char *>();
        const size_t length = name ? strnlen(name, 24) : 0;
        if (length < 2 || length == 24 || name[0] < 'A' || name[0] > 'Z') {
            error = "Invalid Unicore output name"; return false;
        }
        for (const char *c = name; *c; ++c)
            if (!(*c >= 'A' && *c <= 'Z') && !(*c >= '0' && *c <= '9')) {
                error = "Invalid Unicore output name"; return false;
            }
        names.emplace_back(name);
    }
    DynamicJsonDocument result(VERIFICATION_DOCUMENT_SIZE);
    result["command_id"] = commandId;
    JsonObject details = result.createNestedObject("details");
    details["kind"] = "Unicorecomm";
    details["expected_logs"].set(expectedLogs);
    JsonArray comparedPorts = details.createNestedArray("compare_ports");
    JsonObject fixedDetails = details.createNestedObject("fixed");
    JsonObject expectedPosition = fixedDetails.createNestedObject("expected");
    expectedPosition["mode"] = "Fixed";
    expectedPosition["self_position"].set(payload["coords"]);
    JsonObject actualPosition = fixedDetails.createNestedObject("actual");
    JsonObject groups = result.createNestedObject("groups");
    for (const char *name : {"fixed", "diagnostics", "rtcm"}) {
        JsonObject group = groups.createNestedObject(name);
        group["checked"] = 0; group["mismatched"] = 0; group["unread"] = 0;
    }
    String mode, modeError;
    bool positionRead = UnicoreReadback::readMode(mode, modeError);
    if (positionRead) actualPosition["mode"] = mode;
    if (positionRead && String(UnicoreReadback::modeKind(mode)) != "base") {
        groups["fixed"]["checked"] = 1;
        groups["fixed"]["mismatched"] = 1;
    } else if (positionRead) {
        double lat = 0, lon = 0, alt = 0;
        positionRead = UnicoreConfigVerification::parseFixedMode(mode.c_str(), lat, lon, alt);
        if (positionRead) {
            JsonObject positionDetails = actualPosition.createNestedObject("self_position");
            positionDetails["lat"] = lat;
            positionDetails["lon"] = lon;
            positionDetails["alt"] = alt;
            actualPosition["coordinate_source"] = "MODE";
            const bool matches = std::abs(lat - wanted[0]) <= 1e-9
                && std::abs(lon - wanted[1]) <= 1e-9
                && std::abs(alt - wanted[2]) <= 0.0001;
            groups["fixed"]["checked"] = 1;
            groups["fixed"]["mismatched"] = matches ? 0 : 1;
        } else {
            modeError = "Unicore MODE has no valid Fixed LLA coordinates";
        }
    }
    if (!positionRead) { groups["fixed"]["unread"] = 1; result["mode_error"] = modeError; }
    String rawLogs, outputError;
    std::vector<UnicoreConfigVerification::Log> logs;
    bool logsRead = UnicoreReadback::readLogList(rawLogs, outputError)
        && UnicoreConfigVerification::parseLogs(rawLogs.c_str(), rawLogs.length(), logs);
    details["logs_state"] = logsRead ? "ok" : "unread";
    JsonArray observedLogs = details.createNestedArray("logs");
    if (logsRead) for (const auto &log : logs) {
        JsonObject observed = observedLogs.createNestedObject();
        observed["port"] = String("COM") + log.port;
        observed["name"] = log.name;
        if (log.period > 0) observed["period"] = log.period;
        else observed["period"] = nullptr;
    }
    const uint8_t comparePort = logsRead ? UnicoreConfigVerification::comparisonPort(selectedPort ? selectedPort : GNSS_UNICORE_PORT, logs) : 0;
    if (comparePort) {
        comparedPorts.add(String("COM") + comparePort);
        details["port_source"] = selectedPort ? "user_selection" : GNSS_UNICORE_PORT >= 1 && GNSS_UNICORE_PORT <= 3 ? "firmware_config" : "log_list_unique";
    } else if (logsRead) {
        logsRead = false;
        result["note"] = logs.empty()
            ? "Danh sách bản tin UM trống; chưa xác định cổng để đối chiếu."
            : "UM có bản tin trên nhiều cổng COM; cần khai báo cổng nối GNSS để đối chiếu.";
    }
    if (logsRead) {
        for (const String &name : names) {
            JsonObject group = groups[name.startsWith("RTCM") ? "rtcm" : "diagnostics"];
            group["checked"] = group["checked"].as<unsigned>() + 1;
            bool matches = false;
            for (const auto &log : logs)
                if (log.port == comparePort && name == log.name && std::abs(log.period - 1) <= 1e-6) matches = true;
            if (!matches) group["mismatched"] = group["mismatched"].as<unsigned>() + 1;
        }
        for (const auto &log : logs) {
            if (log.port != comparePort || log.period == 0) continue;
            bool expected = false;
            for (const String &name : names) if (name == log.name) expected = true;
            if (!expected) {
                JsonObject group = groups[std::strncmp(log.name, "RTCM", 4) ? "diagnostics" : "rtcm"];
                group["checked"] = group["checked"].as<unsigned>() + 1;
                group["mismatched"] = group["mismatched"].as<unsigned>() + 1;
            }
        }
        for (const char *name : {"diagnostics", "rtcm"})
            if (groups[name]["checked"].as<unsigned>() == 0) groups[name]["checked"] = 1;
    } else {
        groups["diagnostics"]["unread"] = groups["rtcm"]["unread"] = 1;
        result["output_error"] = outputError;
    }
    bool mismatch = false, unread = false;
    for (JsonPair group : groups) {
        mismatch |= group.value()["mismatched"].as<unsigned>() > 0;
        unread |= group.value()["unread"].as<unsigned>() > 0;
    }
    result["state"] = unread ? "unread" : mismatch ? "mismatch" : "match";
    if (result.overflowed()) { error = "GNSS verification result overflow"; return false; }
    serializeJson(result, gnssVerification_);
    gnssVerificationMs_ = millis(); markStatusDirty();
    return true;
}

bool BackendAgent::verifyGnssConfig(JsonObjectConst payload, const String &commandId, String &error) {
    gnssVerification_ = static_cast<const char *>(nullptr);
    nvsInventory_ = static_cast<const char *>(nullptr);
    uint8_t expectedMode[40], expectedOutput[UbxConfigVerification::MAX_KEYS * 5];
    const size_t modeLength = decodeBase64(payload["tmode_b64"] | "", expectedMode, sizeof(expectedMode));
    const size_t outputLength = decodeBase64(payload["output_b64"] | "", expectedOutput, sizeof(expectedOutput));
    if (modeLength != 40 || expectedMode[2] != 2 || expectedMode[3] != 1
        || !UbxConfigVerification::validExpected(expectedOutput, outputLength)) {
        error = "Invalid GNSS verification expectation";
        return false;
    }
    uint8_t actualMode[40] = {}, values[UbxConfigVerification::MAX_KEYS] = {};
    String modeError, outputError;
    const bool modeRead = UbloxReadback::readTmode(actualMode, modeError);
    const JsonObjectConst inspection = payload["readback"].as<JsonObjectConst>();
    bool interfaceRead = false;
    uint8_t interfaceEnabled = 0;
    bool outputRead = inspection.isNull()
        && UbloxReadback::readOutputValues(expectedOutput, outputLength, values, outputError);
    DynamicJsonDocument result(VERIFICATION_DOCUMENT_SIZE);
    result["command_id"] = commandId;
    JsonObject details = result.createNestedObject("details");
    details["kind"] = "Ublox";
    JsonObject fixedDetails = details.createNestedObject("fixed");
    describeTmode(fixedDetails.createNestedObject("expected"), expectedMode);
    if (modeRead) describeTmode(fixedDetails.createNestedObject("actual"), actualMode);
    if (!inspection.isNull()) {
        const JsonArrayConst fields = inspection["fields"].as<JsonArrayConst>();
        const JsonArrayConst ports = inspection["ports"].as<JsonArrayConst>();
        if (fields.size() < 3 || fields.size() > UbxConfigVerification::MAX_KEYS || ports.size() != 5) {
            error = "Invalid GNSS readback catalogue"; return false;
        }
        details["fields"].set(fields);
        details["expected"].set(inspection["expected"]);
        details.createNestedArray("compare_ports").add("UART1");
        JsonArray observedPorts = details.createNestedArray("ports");
        unsigned seen = 0;
        for (JsonObjectConst request : ports) {
            const String name = request["name"] | "";
            const int portIndex = name == "I2C" ? 0 : name == "UART1" ? 1 : name == "UART2" ? 2 : name == "USB" ? 3 : name == "SPI" ? 4 : -1;
            if (portIndex < 0 || (seen & (1U << portIndex))) { error = "Invalid GNSS readback port"; return false; }
            seen |= 1U << portIndex;
            uint8_t records[UbxConfigVerification::MAX_KEYS * 5], observed[UbxConfigVerification::MAX_KEYS];
            const size_t length = decodeBase64(request["output_b64"] | "", records, sizeof(records));
            if (length != fields.size() * 5 || !UbxConfigVerification::validExpected(records, length)) {
                error = "Invalid GNSS readback keys"; return false;
            }
            String readError;
            const bool read = UbloxReadback::readOutputValues(records, length, observed, readError);
            JsonObject port = observedPorts.createNestedObject();
            port["name"] = name;
            port["state"] = read ? "ok" : "unread";
            if (read) {
                JsonArray array = port.createNestedArray("values");
                for (size_t i = 0; i < fields.size(); ++i) {
                    array.add(observed[i]);
                    if (UbxBaseStatus::u32(records + i * 5) == UbxConfigVerification::enabledKey(portIndex)) {
                        port["enabled"] = observed[i];
                        if (portIndex == 1) { interfaceRead = true; interfaceEnabled = observed[i]; }
                    }
                }
            } else port["error"] = readError;
            if (portIndex == 1 || portIndex == 2) {
                uint32_t baud = 0;
                String baudError;
                if (UbloxReadback::readBaud(portIndex, baud, baudError)) port["baud"] = baud;
                else port["baud"] = nullptr;
            }
            if (name == "UART1") {
                // Match by key, never by the ordering of a catalogue supplied over the wire.
                outputRead = read;
                for (size_t i = 0; read && i < outputLength; i += 5) {
                    bool found = false;
                    for (size_t j = 0; j < length; j += 5) if (UbxBaseStatus::u32(expectedOutput + i) == UbxBaseStatus::u32(records + j)) {
                        values[i / 5] = observed[j / 5]; found = true; break;
                    }
                    if (!found) outputRead = false;
                }
                if (!outputRead) outputError = readError.length() ? readError : "Missing UART1 readback keys";
            }
        }
    }
    JsonObject groups = result.createNestedObject("groups");
    JsonObject fixed = groups.createNestedObject("fixed");
    fixed["checked"] = modeRead ? 1 : 0;
    fixed["mismatched"] = modeRead && !UbxConfigVerification::fixedMatches(expectedMode, actualMode, 40) ? 1 : 0;
    fixed["unread"] = modeRead ? 0 : 1;
    for (const char *name : {"protocols", "diagnostics", "rtcm"}) {
        JsonObject group = groups.createNestedObject(name);
        group["checked"] = 0; group["mismatched"] = 0; group["unread"] = 0;
    }
    for (size_t i = 0; i < outputLength; i += 5) {
        const uint32_t key = UbxBaseStatus::u32(expectedOutput + i);
        const char *name = key >> 28 == 1 ? "protocols" : key >= 0x209102BD ? "rtcm" : "diagnostics";
        JsonObject group = groups[name];
        const char *count = outputRead ? "checked" : "unread";
        group[count] = group[count].as<unsigned>() + 1;
        if (outputRead && values[i / 5] != expectedOutput[i + 4])
            group["mismatched"] = group["mismatched"].as<unsigned>() + 1;
    }
    if (!inspection.isNull()) {
        JsonObject protocols = groups["protocols"];
        const char *count = interfaceRead ? "checked" : "unread";
        protocols[count] = protocols[count].as<unsigned>() + 1;
        if (interfaceRead && interfaceEnabled != 1) protocols["mismatched"] = protocols["mismatched"].as<unsigned>() + 1;
    }
    bool mismatch = false, unread = false;
    for (JsonPair group : groups) {
        mismatch |= group.value()["mismatched"].as<unsigned>() > 0;
        unread |= group.value()["unread"].as<unsigned>() > 0 || !group.value()["checked"].as<unsigned>();
    }
    result["state"] = unread ? "unread" : mismatch ? "mismatch" : "match";
    if (!modeRead) result["mode_error"] = modeError;
    if (!outputRead) result["output_error"] = outputError;
    if (result.overflowed()) { error = "GNSS verification result overflow"; return false; }
    serializeJson(result, gnssVerification_);
    gnssVerificationMs_ = millis();
    markStatusDirty();
    return true; // Command completion is separate from match/mismatch/unread.
}

void BackendAgent::saveSurveyResult(JsonObjectConst bestPosition, const String &mode) {
    if (!surveyInMonitoring_ || !surveyInReadbackReady_ || !surveyInReadbackHasPosition_
        || (!surveyInResultPending_ && surveyInResult_.length() > 2)) return;
    DynamicJsonDocument savedSurvey(1536);
    savedSurvey["chip"] = sensorType_;
    savedSurvey["bestpos"].set(bestPosition);
    savedSurvey["mode_raw"] = mode;
    savedSurvey["recorded_at"] = epochString().toInt();
    surveyInResult_.clear();
    serializeJson(savedSurvey, surveyInResult_);
    surveyInResultPending_ = false;
    saveStatePreferences();
}

bool BackendAgent::refreshReceiverReadback(String &error) {
    lastReceiverPollMs_ = millis();
    surveyInReadbackReady_ = false;
    surveyInReadbackHasPosition_ = false;
    return sensorType_ == "Ublox" ? refreshUbloxReadback(error) : refreshUnicoreReadback(error);
}

bool BackendAgent::refreshUbloxReadback(String &error) {
    UbxBaseStatus status;
    const bool success = UbloxReadback::readBaseStatus(status, error);
    DynamicJsonDocument doc(1536);
    doc["state"] = success ? "ok" : "error";
    doc["chip"] = "Ublox";
    doc["revision"] = ++receiverBaseReadbackRevision_;
    doc["query"] = "CFG-TMODE3 / NAV-SVIN";
    const char *kind = status.mode == 1 ? "survey_in" : status.mode == 2 ? "base" : "rover";
    const String mode = status.mode == 1 ? "Survey-In (NAV-SVIN)" : status.mode == 2 ? "Fixed LLA (CFG-TMODE3)" : "Rover (CFG-TMODE3)";
    doc["mode_kind"] = success ? kind : "unknown";
    doc["mode_raw"] = mode;
    surveyInReadbackHasPosition_ = success && status.hasPosition;
    surveyInReadbackReady_ = success && status.valid && !status.active && status.hasPosition;
    if (success) {
        if (status.mode == 1) {
            Serial.printf("[GNSS][READBACK] UBX Survey-In elapsed=%lus/%lus observations=%lu accuracy=%.3fm active=%u valid=%u\n",
                          static_cast<unsigned long>(status.duration),
                          static_cast<unsigned long>(status.targetSeconds),
                          static_cast<unsigned long>(status.observations), status.accuracy,
                          static_cast<unsigned int>(status.active), static_cast<unsigned int>(status.valid));
        } else {
            Serial.printf("[GNSS][READBACK] UBX mode=%s; position=%s\n",
                          status.mode == 2 ? "Fixed" : "Rover",
                          status.hasPosition ? "valid" : "unavailable");
        }
        surveyInMonitoring_ = status.mode == 1;
        if (surveyInMonitoring_) surveyInTargetSeconds_ = status.targetSeconds;
        doc["elapsed_seconds"] = status.duration;
        doc["observations"] = status.observations;
        doc["mean_accuracy_m"] = status.accuracy;
        doc["active"] = status.active;
        JsonObject position = doc.createNestedObject("bestpos");
        position["state"] = status.hasPosition ? "ok" : "unavailable";
        if (status.hasPosition) {
            position["message"] = "UBX";
            position["position_type"] = surveyInReadbackReady_ ? "FIXEDPOS" : "SURVEY-IN-ESTIMATE";
            position["datum"] = "WGS84";
            position["altitude_reference"] = "ELLIPSOID";
            char value[32];
            snprintf(value, sizeof(value), "%.11f", status.latitude); position["lat"] = value;
            snprintf(value, sizeof(value), "%.11f", status.longitude); position["lon"] = value;
            snprintf(value, sizeof(value), "%.4f", status.height); position["alt"] = value;
            if (surveyInReadbackReady_) saveSurveyResult(position, mode);
        }
    } else {
        Serial.printf("[GNSS][READBACK][ERROR] UBX: %s\n", error.c_str());
        doc["error"] = error;
    }
    reconcileBaseStartupReadback(doc.as<JsonObjectConst>());
    receiverBaseReadback_.clear(); serializeJson(doc, receiverBaseReadback_);
    receiverBaseReadbackMs_ = millis(); markStatusDirty();
    return success;
}

bool BackendAgent::refreshUnicoreReadback(String &error) {
    unicoreQualityMs_ = 0;
    String mode;
    const bool success = UnicoreReadback::readMode(mode, error);
    DynamicJsonDocument readbackDoc(1024);
    readbackDoc["state"] = success ? "ok" : "error";
    readbackDoc["chip"] = "Unicorecomm";
    readbackDoc["revision"] = ++receiverBaseReadbackRevision_;
    readbackDoc["mode_raw"] = mode;
    readbackDoc["mode_kind"] = success ? UnicoreReadback::modeKind(mode) : "unknown";
    readbackDoc["query"] = "MODE";
    if (success) {
        if (String(UnicoreReadback::modeKind(mode)) == "survey_in") {
            if (!surveyInMonitoring_) {
                surveyInMonitoring_ = true;
                surveyInStartedMs_ = 0;
                surveyInTargetSeconds_ = 0;
            }
            unsigned long target = 0;
            if (std::sscanf(mode.c_str(), "MODE BASE TIME %lu", &target) == 1)
                surveyInTargetSeconds_ = target;
        } else {
            surveyInMonitoring_ = false;
        }
        JsonObject bestPosition = readbackDoc.createNestedObject("bestpos");
        String positionError;
        if (!UnicoreReadback::readBestPosition(bestPosition, positionError)) {
            surveyInReadbackHasPosition_ = false;
            bestPosition["state"] = "unavailable";
            bestPosition["error"] = positionError;
        } else {
            surveyInReadbackHasPosition_ = true;
            if (!bestPosition["horizontal_sigma_m"].isNull()) {
                unicoreHorizontalSigma_ = bestPosition["horizontal_sigma_m"].as<float>();
                unicoreQualityMs_ = millis();
            }
            surveyInReadbackReady_ = String(bestPosition["position_type"] | "") == "FIXEDPOS";
        }
        saveSurveyResult(bestPosition, mode);

    } else {
        surveyInReadbackHasPosition_ = false;
        readbackDoc["error"] = error;
    }
    reconcileBaseStartupReadback(readbackDoc.as<JsonObjectConst>());
    receiverBaseReadback_.clear();
    serializeJson(readbackDoc, receiverBaseReadback_);
    receiverBaseReadbackMs_ = millis();
    markStatusDirty();
    return success;
}

void BackendAgent::pollGnssStatus() {
    if ((sensorType_ != "Unicorecomm" && sensorType_ != "Ublox") || workflowActive() || stationLock_.all) return;
    const uint32_t now = millis();
    if ((lastReceiverPollMs_ == 0 && now >= 5000UL)
        || (lastReceiverPollMs_ != 0 && now - lastReceiverPollMs_ >= 30000UL)) {
        String error;
        refreshReceiverReadback(error);
        if (sensorType_ == "Ublox") {
            UbxNavigation navigation;
            if (UbloxReadback::readNavigation(navigation, error)) {
                navigation_ = navigation;
                navigationMs_ = millis();
                markStatusDirty();
            }
            UbxDop dop;
            if (UbloxReadback::readDop(dop, error)) {
                dop_ = dop;
                dopMs_ = millis();
                markStatusDirty();
            }
            if (UbloxReadback::readSatellites(satelliteReadback_, error)) {
                satelliteReadbackMs_ = millis();
                markStatusDirty();
            }
            versionReadback_ = "";
            versionReadbackError_ = "";
            error = "";
            if (!UbloxReadback::readVersion(versionReadback_, error)) versionReadbackError_ = error.substring(0, 160);
            receiverInfoMs_ = millis();
            markStatusDirty();
        }
    }

    // MODE/BESTNAV are read-only receiver logs; do not request NMEA output.
}

bool BackendAgent::writeUnicoreCommand(const uint8_t *data, size_t length, String &error) {
    UnicoreReply reply(data, length);
    if (!reply.key[0]) { error = "invalid Unicore ASCII command"; return false; }
    if (!rtcmBufferMutex || xSemaphoreTake(rtcmBufferMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) != pdTRUE) {
        error = "GNSS command port is busy";
        return false;
    }
    if (!gnssSerialMutex || xSemaphoreTake(gnssSerialMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) != pdTRUE) {
        xSemaphoreGive(rtcmBufferMutex);
        error = "GNSS reader is busy";
        return false;
    }
    // Own UART RX until the matching reply is read; the stream task must not eat the ACK.
    while (Serial1.available()) Serial1.read();
    const bool written = Serial1.write(data, length) == length;
    Serial1.flush();
    const uint32_t started = millis();
    bool matched = false;
    while (written && millis() - started < 8000UL && !matched) {
        while (Serial1.available()) {
            if (reply.feed(static_cast<uint8_t>(Serial1.read()))) { matched = true; break; }
        }
        if (!matched) vTaskDelay(pdMS_TO_TICKS(5));
    }
    xSemaphoreGive(gnssSerialMutex);
    xSemaphoreGive(rtcmBufferMutex);
    if (!written) error = String("Unicore UART write failed: ") + reply.key;
    else if (!matched) error = String("Unicore response timeout: ") + reply.key;
    else if (reply.state != UnicoreReply::State::Ok) error = String("Unicore rejected command: ") + reply.line;
    else {
        Serial.printf("[GNSS][UNICORE] ACK OK: %s\n", reply.line);
        if (!std::strcmp(reply.key, "mode")) {
            String command;
            command.reserve(length);
            for (size_t i = 0; i < length; ++i) command += static_cast<char>(data[i]);
            command.trim();
            command.toLowerCase();
            unsigned long target = 0;
            surveyInMonitoring_ = std::sscanf(command.c_str(), "mode base time %lu", &target) == 1
                && target > 0;
            surveyInStartedMs_ = surveyInMonitoring_ ? millis() : 0;
            surveyInTargetSeconds_ = surveyInMonitoring_ ? target : 0;
            if (surveyInMonitoring_) surveyInResultPending_ = true;
        }
        // A reset can ACK before the receiver actually reboots. Never retry it blindly.
        if (!std::strcmp(reply.key, "freset") || !std::strcmp(reply.key, "reset")) delay(2000);
        return true;
    }
    Serial.printf("[GNSS][UNICORE][ERROR] %s\n", error.c_str());
    return false;
}

bool BackendAgent::writeGnssCommands(const GnssCmdBuilder::CommandList &commands) {
    if (commands.empty()) return false;
    for (const auto &command : commands) {
        uint32_t delayMs = 0;
        if (delayMarker(command.data(), command.size(), delayMs)) {
            delay(delayMs);
            continue;
        }
        if (sensorType_ == "Unicorecomm") {
            String error;
            if (!writeUnicoreCommand(command.data(), command.size(), error)) return false;
            continue;
        }
        if (!rtcmBufferMutex
            || xSemaphoreTake(rtcmBufferMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) != pdTRUE) return false;
        const size_t written = Serial1.write(command.data(), command.size());
        Serial1.flush();
        xSemaphoreGive(rtcmBufferMutex);
        if (written != command.size()) return false;
        delay(100);
    }
    return true;
}

bool BackendAgent::applyRoverMode() {
    if (sensorType_ != "Ublox" && sensorType_ != "Unicorecomm") return false;
    Serial.printf("[AUTO BASE] Gui lenh Rover + GGA UART cho chip %s\n", sensorType_.c_str());
    if (sensorType_ == "Unicorecomm") {
        return writeGnssCommands(GnssCmdBuilder::buildRoverCommand(GnssCmdBuilder::GnssSensorType::Unicore));
    }
    if (!rtcmBufferMutex || xSemaphoreTake(rtcmBufferMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) != pdTRUE) return false;
    if (!gnssSerialMutex || xSemaphoreTake(gnssSerialMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) != pdTRUE) {
        xSemaphoreGive(rtcmBufferMutex);
        return false;
    }
    // Own both directions until ACK/readback completes; the stream task must
    // not consume the replies before the workflow can inspect them.
    auto exchange = [](const GnssCmdBuilder::Command &command, bool poll) {
        while (Serial1.available()) Serial1.read();
        if (Serial1.write(command.data(), command.size()) != command.size()) return false;
        Serial1.flush();
        UbxReply reply;
        const uint32_t started = millis();
        while (millis() - started < 1500UL) {
            while (Serial1.available()) {
                if (!reply.feed(static_cast<uint8_t>(Serial1.read()))) continue;
                if (poll && reply.frame[2] == 0x06 && reply.frame[3] == 0x71 && reply.length == 48) {
                    const uint8_t mode = reply.frame[8];
                    Serial.printf("[AUTO BASE] UBX-CFG-TMODE3 readback mode=%u (0=Rover)\n", mode);
                    return mode == 0;
                }
                if (!poll && reply.length == 10 && reply.frame[2] == 0x05
                    && (reply.frame[3] == 0x00 || reply.frame[3] == 0x01)
                    && reply.frame[6] == command[2] && reply.frame[7] == command[3]) {
                    Serial.printf("[AUTO BASE] UBX %02X/%02X: %s\n", command[2], command[3],
                                  reply.frame[3] == 1 ? "ACK" : "NAK (chip tu choi lenh)");
                    return reply.frame[3] == 1;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        Serial.printf("[AUTO BASE] UBX %02X/%02X: khong co %s tu chip\n", command[2], command[3], poll ? "readback" : "ACK");
        return false;
    };
    for (const auto &command : GnssCmdBuilder::buildRoverCommand(GnssCmdBuilder::GnssSensorType::Ublox)) {
        // Unsupported optional port keys must not block legacy CFG commands.
        exchange(command, false);
    }
    const bool roverConfirmed = exchange({0xB5, 0x62, 0x06, 0x71, 0, 0, 0x77, 0x6B}, true);
    xSemaphoreGive(gnssSerialMutex);
    xSemaphoreGive(rtcmBufferMutex);
    if (!roverConfirmed) {
        Serial.println("[AUTO BASE][ERROR] Chua xac nhan chip o Rover; dung truoc khi ket noi caster");
        return false;
    }
    const uint32_t waiting = millis();
    while (millis() - waiting < 10000UL && !workflowCancelRequested_) {
        const auto snapshot = getGnssTelemetrySnapshot();
        if (snapshot.lastGgaMs != 0 && static_cast<int32_t>(snapshot.lastGgaMs - waiting) > 0) {
            Serial.printf("[AUTO BASE] Rover da xac nhan, GGA UART: %s\n", snapshot.lastGga.data());
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    Serial.println("[AUTO BASE][ERROR] Chip o Rover nhung khong co GGA moi trong 10s");
    return false;
}
