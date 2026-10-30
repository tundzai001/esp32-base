#include "functions/Backend_Agent.h"
#include "Backend_Agent_Internal.h"
#include "Prog_Config.h"
#include "functions/NTRIP_Handler_IP.h"
#include "functions/RTCM_Receiver.h"
#include "functions/Base_Sample.h"
#include <cmath>
#include <cstring>
#include <vector>

#include "functions/Ubx_Reply.h"
#include "functions/Unicore_Reply.h"
#include <cstdio>

using namespace BackendAgentInternal;

extern SemaphoreHandle_t tcpStreamMutex;
extern SemaphoreHandle_t rtcmBufferMutex;

void BackendAgent::setWorkflowProgress(const String &phase, const uint8_t step,
                                       const uint8_t total, const String &details) {
    DynamicJsonDocument progress(CONFIG_DOCUMENT_SIZE);
    progress["phase"] = phase;
    progress["step"] = step;
    progress["total_steps"] = total;
    progress["updated_at"] = epochString().toInt();

    if (!details.isEmpty()) {
        DynamicJsonDocument extra(4096);
        if (deserializeJson(extra, details) == DeserializationError::Ok
            && extra.is<JsonObject>()) {
            for (JsonPairConst pair : extra.as<JsonObjectConst>()) {
                progress[pair.key().c_str()] = pair.value();
            }
        }
    }
    autoBaseProgress_.clear();
    serializeJson(progress, autoBaseProgress_);
    markStatusDirty();
}

bool BackendAgent::startWorkflow(const String &kind, const String &payload) {
    if (workflowTask_ != nullptr) return false;
    workflowKind_ = kind;
    workflowPayload_ = payload;
    workflowCancelRequested_ = false;
    const BaseType_t created = xTaskCreatePinnedToCore(
        &BackendAgent::workflowTaskThunk,
        "Backend Workflow",
        12288,
        this,
        1,
        &workflowTask_,
        1);
    if (created != pdPASS) {
        workflowKind_ = String();
        workflowPayload_ = String();
        workflowTask_ = nullptr;
        return false;
    }
    return true;
}

void BackendAgent::requestWorkflowStop() {
    if (workflowTask_ != nullptr) workflowCancelRequested_ = true;
}

void BackendAgent::workflowTaskThunk(void *parameter) {
    BackendAgent *agent = static_cast<BackendAgent *>(parameter);
    if (agent) {
        agent->runWorkflow();
        agent->workflowPayload_ = String();
        agent->workflowKind_ = String();
        agent->workflowCancelRequested_ = false;
        agent->workflowTask_ = nullptr;
    }
    vTaskDelete(nullptr);
}

bool BackendAgent::restartConfiguredNtrip() {
    if (!tcpStreamMutex
        || xSemaphoreTake(tcpStreamMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS * 4)) != pdTRUE) {
        return false;
    }
    stopNtripRover();
    setupNTRIP();
    connectNTRIP();
    xSemaphoreGive(tcpStreamMutex);
    return true;
}

int BackendAgent::loopNtripRoverLocked() {
    if (!tcpStreamMutex
        || xSemaphoreTake(tcpStreamMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) != pdTRUE) {
        // Contention is not a caster disconnect; let the workflow retry on
        // its next pass without touching a TinyGSM socket concurrently.
        return 200;
    }
    const int result = loopNtripRover();
    xSemaphoreGive(tcpStreamMutex);
    return result;
}

bool BackendAgent::stopNtripRoverLocked() {
    if (!tcpStreamMutex
        || xSemaphoreTake(tcpStreamMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS * 4)) != pdTRUE) {
        return false;
    }
    stopNtripRover();
    xSemaphoreGive(tcpStreamMutex);
    return true;
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
            Serial.printf("[AUTO BASE] Rover da xac nhan, GGA UART: %s\n", snapshot.lastGga);
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    Serial.println("[AUTO BASE][ERROR] Chip o Rover nhung khong co GGA moi trong 10s");
    return false;
}

bool BackendAgent::applyBaseCoordinates(const double latitude, const double longitude,
                                        const double altitude) {
    if (!std::isfinite(latitude) || !std::isfinite(longitude) || !std::isfinite(altitude)
        || latitude < -90.0 || latitude > 90.0
        || longitude < -180.0 || longitude > 180.0) {
        return false;
    }
    if (sensorType_ != "Ublox" && sensorType_ != "Unicorecomm") return false;
    GnssCmdBuilder::GnssOptions options;
    options.ports = {"UART1"};
    return writeGnssCommands(GnssCmdBuilder::buildBaseFixedLlaCommand(
        latitude, longitude, altitude, 0.01f,
        sensorType_ == "Ublox" ? GnssCmdBuilder::GnssSensorType::Ublox
                                : GnssCmdBuilder::GnssSensorType::Unicore,
        options));
}

void BackendAgent::runWorkflow() {
    DynamicJsonDocument document(COMMAND_DOCUMENT_SIZE);
    if (deserializeJson(document, workflowPayload_) != DeserializationError::Ok
        || !document.is<JsonObject>()) {
        setAgentState(normalAgentState());
        setWorkflowProgress("failed", 0, 4, "{\"reason\":\"invalid workflow payload\"}");
        saveStatePreferences();
        return;
    }

    const JsonObject payload = document.as<JsonObject>();
    const String kind = workflowKind_;
    if (kind == "auto_base") {
        runAutoBaseWorkflow(payload);
    } else if (kind == "reference_check") {
        runReferenceCheckWorkflow(payload);
    } else {
        setAgentState(normalAgentState());
        setWorkflowProgress("failed", 0, 4, "{\"reason\":\"unknown workflow\"}");
        saveStatePreferences();
    }
}

bool BackendAgent::runAutoBaseWorkflow(JsonObject payload) {
    const String host = payload["ip"] | String();
    const uint16_t port = static_cast<uint16_t>(payload["port"] | 0);
    const String username = payload["user"] | String();
    const String password = payload["password"] | String();
    const String mountpoint = payload["mountpoint"] | String();
    const uint8_t ntripVersion = static_cast<uint8_t>(payload["ntrip_version"] | 1);
    const uint32_t timeoutSeconds = constrain(
        static_cast<uint32_t>(payload["timeout"] | 3600UL), 30UL, 86400UL);
    const uint8_t fixedStreakRequired = static_cast<uint8_t>(constrain(
        static_cast<int>(payload["fixed_streak_seconds"] | 5), 1, 60));

    auto finish = [&](const bool success, const bool cancelled, const String &reason) {
        // restartConfiguredNtrip() stops the temporary rover session while
        // holding tcpStreamMutex, then restores the configured source streams.
        const bool transportRestored = restartConfiguredNtrip();
        setAgentState(normalAgentState());
        DynamicJsonDocument details(1024);
        if (!reason.isEmpty()) details["reason"] = reason;
        if (!transportRestored) details["cleanup_error"] = "could not restore configured NTRIP transport";
        String serialized;
        serializeJson(details, serialized);
        setWorkflowProgress(
            success ? "completed" : (cancelled ? "stopped" : "failed"),
            success ? 4 : 2, 4, serialized);
        Serial.printf("[AUTO BASE] KET QUA: %s; %s\n",
                      success ? "THANH CONG" : (cancelled ? "DA DUNG" : "THAT BAI"), reason.c_str());
        saveStatePreferences();
    };

    if (host.isEmpty() || username.isEmpty() || password.isEmpty() || mountpoint.isEmpty()
        || port == 0 || port > 65535) {
        finish(false, false, "missing or invalid CORS connection fields");
        return false;
    }

    setAgentState("auto_setup_rover");
    setWorkflowProgress("set_rover", 1, 4);
    if (!applyRoverMode()) {
        finish(false, false, "cannot configure GNSS rover mode");
        return false;
    }

    bool roverStarted = false;
    if (tcpStreamMutex
        && xSemaphoreTake(tcpStreamMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE) {
        stopNtrip();
        roverStarted = startNtripRover(host, port, username, password, mountpoint, ntripVersion);
        xSemaphoreGive(tcpStreamMutex);
    }
    if (!roverStarted) {
        finish(false, false, "cannot connect to CORS rover mountpoint");
        return false;
    }

    setAgentState("auto_setup_wait_fix");
    setWorkflowProgress("wait_fix", 2, 4);
    const uint32_t waitStartedMs = millis();
    uint32_t nextSecondMs = waitStartedMs;
    uint8_t fixedStreak = 0;
    bool fixed = false;
    while (millis() - waitStartedMs < timeoutSeconds * 1000UL) {
        if (workflowCancelRequested_) {
            finish(false, true, "cancelled by user");
            return false;
        }
        if (loopNtripRoverLocked() == 504) {
            finish(false, false, "CORS rover connection lost");
            return false;
        }

        const uint32_t now = millis();
        if (static_cast<int32_t>(now - nextSecondMs) >= 0) {
            nextSecondMs = now + 1000UL;
            const GnssTelemetrySnapshot snapshot = getGnssTelemetrySnapshot();
            const bool usable = freshRtkSample(now, snapshot.lastGgaMs, snapshot.fixStatus);
            fixedStreak = usable ? static_cast<uint8_t>(fixedStreak + 1) : 0;
            fixed = fixedStreak >= fixedStreakRequired;
            DynamicJsonDocument details(1024);
            details["timeout"] = timeoutSeconds;
            details["elapsed"] = (now - waitStartedMs) / 1000UL;
            details["fix_status"] = snapshot.fixStatus;
            details["fixed_streak"] = fixedStreak;
            details["fixed_streak_required"] = fixedStreakRequired;
            String serialized;
            serializeJson(details, serialized);
            setWorkflowProgress("wait_fix", 2, 4, serialized);
            if (fixed) break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (!fixed) {
        finish(false, false, "RTK fix timeout");
        return false;
    }

    setAgentState("auto_setup_averaging");
    const int configuredSamples = payload["samples"] | 60;
    constexpr uint16_t samplesNeeded = 60;
    constexpr uint32_t averagingSeconds = 60;
    setWorkflowProgress("averaging", 3, 4);
    std::vector<double> latitudes;
    std::vector<double> longitudes;
    std::vector<double> altitudes;
    latitudes.reserve(samplesNeeded);
    longitudes.reserve(samplesNeeded);
    altitudes.reserve(samplesNeeded);

    const uint32_t averageStartedMs = millis();
    uint32_t lastSampleGgaMs = 0;
    nextSecondMs = averageStartedMs;
    while (millis() - averageStartedMs < averagingSeconds * 1000UL
           && latitudes.size() < samplesNeeded) {
        if (workflowCancelRequested_) {
            finish(false, true, "cancelled by user");
            return false;
        }
        if (loopNtripRoverLocked() == 504) {
            finish(false, false, "CORS rover connection lost while averaging");
            return false;
        }

        const uint32_t now = millis();
        if (static_cast<int32_t>(now - nextSecondMs) >= 0) {
            nextSecondMs = now + 1000UL;
            const GnssTelemetrySnapshot snapshot = getGnssTelemetrySnapshot();
            const bool usable = freshRtkSample(now, snapshot.lastGgaMs, snapshot.fixStatus);
            if (usable && snapshot.lastGgaMs != lastSampleGgaMs
                && std::isfinite(snapshot.latitude) && std::isfinite(snapshot.longitude)
                && std::isfinite(snapshot.altitude)
                && snapshot.latitude >= -90.0 && snapshot.latitude <= 90.0
                && snapshot.longitude >= -180.0 && snapshot.longitude <= 180.0) {
                lastSampleGgaMs = snapshot.lastGgaMs;
                latitudes.push_back(snapshot.latitude);
                longitudes.push_back(snapshot.longitude);
                altitudes.push_back(snapshot.altitude);
            }
            DynamicJsonDocument details(1200);
            details["samples_collected"] = latitudes.size();
            details["samples_target"] = samplesNeeded;
            details["averaging_seconds"] = averagingSeconds;
            details["elapsed"] = (now - averageStartedMs) / 1000UL;
            details["fix_status"] = snapshot.fixStatus;
            if (configuredSamples != samplesNeeded) {
                details["requested_samples_overridden"] = configuredSamples;
            }
            if (usable) {
                JsonObject latest = details.createNestedObject("latest_coord");
                latest["lat"] = snapshot.latitude;
                latest["lon"] = snapshot.longitude;
                latest["alt"] = snapshot.altitude;
            }
            String serialized;
            serializeJson(details, serialized);
            setWorkflowProgress("averaging", 3, 4, serialized);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (latitudes.size() < 15) {
        finish(false, false, "insufficient valid RTK samples");
        return false;
    }

    double rawLatitude = 0.0;
    double rawLongitude = 0.0;
    double rawAltitude = 0.0;
    for (size_t index = 0; index < latitudes.size(); ++index) {
        rawLatitude += latitudes[index];
        rawLongitude += longitudes[index];
        rawAltitude += altitudes[index];
    }
    rawLatitude /= latitudes.size();
    rawLongitude /= longitudes.size();
    rawAltitude /= altitudes.size();

    double centralMeridianDeg = 105.0;
    double scale = 0.9999;
    String provinceName = "Khong xac dinh (fallback theo kinh do)";
    String provinceCode = "UNKNOWN";
    const String requestedProvince = payload["province_code"] | String();
    if (!requestedProvince.isEmpty()) {
        if (!getProvinceProjection(requestedProvince, centralMeridianDeg, scale, provinceName)) {
            finish(false, false, "invalid province_code");
            return false;
        }
        provinceCode = requestedProvince;
    } else if (payload["l0_deg"].isNull()) {
        static constexpr double candidates[] = {
            103.0, 104.0, 104.5, 104.75, 105.0, 105.5, 105.75,
            106.0, 106.5, 107.0, 107.25, 107.75, 108.0, 108.25, 108.5,
        };
        double bestDistance = 1e9;
        for (const double candidate : candidates) {
            const double distance = fabs(candidate - rawLongitude);
            if (distance < bestDistance) {
                bestDistance = distance;
                centralMeridianDeg = candidate;
            }
        }
    }
    if (!payload["l0_deg"].isNull()) centralMeridianDeg = payload["l0_deg"].as<double>();
    if (!payload["k0"].isNull()) scale = payload["k0"].as<double>();

    bool transformed = false;
    double finalLatitude = rawLatitude;
    double finalLongitude = rawLongitude;
    double finalAltitude = rawAltitude;
    double northing = 0.0;
    double easting = 0.0;
    const bool enableTransform = payload["enable_itrf_vn2000_transform"] | true;
    if (enableTransform) {
        transformed = transformToVn2000(rawLatitude, rawLongitude, rawAltitude,
                                         centralMeridianDeg, scale, finalLatitude,
                                         finalLongitude, finalAltitude, northing, easting);
    }

    setAgentState("auto_setup_base");
    DynamicJsonDocument baseProgress(2048);
    baseProgress["averaged_coord"]["lat"] = finalLatitude;
    baseProgress["averaged_coord"]["lon"] = finalLongitude;
    baseProgress["averaged_coord"]["alt"] = finalAltitude;
    baseProgress["sample_count"] = latitudes.size();
    baseProgress["transform_applied"] = transformed;
    baseProgress["projection_source"] = requestedProvince.isEmpty() ? "lon_fallback" : "manual_province";
    baseProgress["province_code"] = provinceCode;
    baseProgress["province_name"] = provinceName;
    baseProgress["central_meridian_deg"] = centralMeridianDeg;
    baseProgress["k0"] = scale;
    String baseProgressJson;
    serializeJson(baseProgress, baseProgressJson);
    setWorkflowProgress("set_base", 4, 4, baseProgressJson);

    if (!stopNtripRoverLocked()) {
        finish(false, false, "cannot acquire transport lock to stop CORS rover session");
        return false;
    }
    if (!applyBaseCoordinates(finalLatitude, finalLongitude, finalAltitude)) {
        finish(false, false, "cannot configure GNSS base mode");
        return false;
    }

    DynamicJsonDocument baseConfig(CONFIG_DOCUMENT_SIZE);
    JsonObject coordinates = baseConfig.createNestedObject("coords");
    coordinates["lat"] = finalLatitude;
    coordinates["lon"] = finalLongitude;
    coordinates["alt"] = finalAltitude;
    baseConfig["altitude_reference"] = "ELLIPSOID";
    baseConfig["base_setup_method"] = "AUTO_CORS";
    baseConfig["auto_configured"] = true;
    baseConfig["fixed_samples_only"] = true;
    baseConfig["itrf_vn2000_transform_applied"] = transformed;
    baseConfig["central_meridian_deg"] = centralMeridianDeg;
    baseConfig["k0"] = scale;
    baseConfig["projection_source"] = requestedProvince.isEmpty() ? "lon_fallback" : "manual_province";
    baseConfig["province_code"] = provinceCode;
    baseConfig["province_name"] = provinceName;
    baseConfig["vn2000_northing"] = northing;
    baseConfig["vn2000_easting"] = easting;
    JsonObject raw = baseConfig.createNestedObject("auto_base_raw_itrf_llh");
    raw["lat"] = rawLatitude;
    raw["lon"] = rawLongitude;
    raw["alt"] = rawAltitude;
    baseConfig_.clear();
    serializeJson(baseConfig, baseConfig_);
    saveStatePreferences();
    const bool transportRestored = restartConfiguredNtrip();
    setAgentState(normalAgentState());

    DynamicJsonDocument completed(2048);
    completed["averaged_coord"]["lat"] = finalLatitude;
    completed["averaged_coord"]["lon"] = finalLongitude;
    completed["averaged_coord"]["alt"] = finalAltitude;
    completed["raw_itrf_coord"]["lat"] = rawLatitude;
    completed["raw_itrf_coord"]["lon"] = rawLongitude;
    completed["raw_itrf_coord"]["alt"] = rawAltitude;
    completed["sample_count"] = latitudes.size();
    completed["fixed_samples_only"] = true;
    completed["transform_applied"] = transformed;
    completed["configured_ntrip_restarted"] = transportRestored;
    completed["projection_source"] = requestedProvince.isEmpty() ? "lon_fallback" : "manual_province";
    completed["province_code"] = provinceCode;
    completed["province_name"] = provinceName;
    completed["central_meridian_deg"] = centralMeridianDeg;
    completed["k0"] = scale;
    String completedJson;
    serializeJson(completed, completedJson);
    setWorkflowProgress("completed", 4, 4, completedJson);
    Serial.printf("[AUTO BASE] KET QUA: THANH CONG; samples=%u; lat=%.10f; lon=%.10f; alt=%.4f\n",
                  static_cast<unsigned>(latitudes.size()), finalLatitude, finalLongitude, finalAltitude);
    saveStatePreferences();
    return true;
}

bool BackendAgent::runReferenceCheckWorkflow(JsonObject payload) {
    JsonObject reference = payload["reference_coords"].as<JsonObject>();
    DynamicJsonDocument storedBase(CONFIG_DOCUMENT_SIZE);
    const bool baseParsed = deserializeJson(storedBase, baseConfig_) == DeserializationError::Ok
        && storedBase.is<JsonObject>();
    JsonObject storedBaseObject = storedBase.as<JsonObject>();
    JsonObject storedCoordinates = storedBaseObject["coords"].as<JsonObject>();
    if (storedCoordinates.isNull()) storedCoordinates = storedBaseObject;

    const double referenceLatitude = reference["lat"] | NAN;
    const double referenceLongitude = reference["lon"] | NAN;
    const double referenceAltitude = reference["alt"] | NAN;
    const double restoreLatitude = storedCoordinates["lat"] | NAN;
    const double restoreLongitude = storedCoordinates["lon"] | NAN;
    const double restoreAltitude = storedCoordinates["alt"] | NAN;
    const String host = payload["ip"] | String();
    const uint16_t port = static_cast<uint16_t>(payload["port"] | 0);
    const String username = payload["user"] | String();
    const String password = payload["password"] | String();
    const String mountpoint = payload["mountpoint"] | String();
    const uint32_t timeoutSeconds = constrain(
        static_cast<uint32_t>(payload["timeout"] | 300UL), 30UL, 86400UL);
    const uint16_t requestedSamples = static_cast<uint16_t>(constrain(
        static_cast<int>(payload["samples"] | 60), 5, 120));

    auto cleanup = [&]() {
        const bool roverStopped = stopNtripRoverLocked();
        if (!roverStopped) return false;
        bool baseRestored = true;
        if (std::isfinite(restoreLatitude) && std::isfinite(restoreLongitude) && std::isfinite(restoreAltitude)) {
            baseRestored = applyBaseCoordinates(restoreLatitude, restoreLongitude, restoreAltitude);
        }
        const bool transportRestored = restartConfiguredNtrip();
        return baseRestored && transportRestored;
    };
    auto finish = [&](const bool success, const bool cancelled, const String &reason) {
        const bool cleanupSucceeded = cleanup();
        const bool completed = success && cleanupSucceeded;
        const bool stopped = cancelled && cleanupSucceeded;
        setAgentState(normalAgentState());
        DynamicJsonDocument details(1024);
        if (!reason.isEmpty()) details["reason"] = reason;
        if (!cleanupSucceeded) details["cleanup_error"] = "could not safely restore base/NTRIP state";
        String serialized;
        serializeJson(details, serialized);
        setWorkflowProgress(
            completed ? "reference_check_completed" : (stopped ? "stopped" : "reference_check_failed"),
            4, 4, serialized);
        saveStatePreferences();
    };

    if (!baseParsed || reference.isNull()
        || !std::isfinite(referenceLatitude) || !std::isfinite(referenceLongitude)
        || !std::isfinite(referenceAltitude) || referenceLatitude < -90.0
        || referenceLatitude > 90.0 || referenceLongitude < -180.0
        || referenceLongitude > 180.0 || !std::isfinite(restoreLatitude)
        || !std::isfinite(restoreLongitude) || !std::isfinite(restoreAltitude)
        || host.isEmpty() || username.isEmpty() || password.isEmpty()
        || mountpoint.isEmpty() || port == 0 || port > 65535) {
        finish(false, false, "missing or invalid reference/CORS configuration");
        return false;
    }

    setAgentState("reference_check_rover");
    setWorkflowProgress("reference_check_set_rover", 1, 4);
    if (!applyRoverMode()) {
        finish(false, false, "cannot configure GNSS rover mode");
        return false;
    }

    bool roverStarted = false;
    if (tcpStreamMutex
        && xSemaphoreTake(tcpStreamMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE) {
        stopNtrip();
        roverStarted = startNtripRover(host, port, username, password, mountpoint, 1);
        xSemaphoreGive(tcpStreamMutex);
    }
    if (!roverStarted) {
        finish(false, false, "cannot connect to CORS reference mountpoint");
        return false;
    }

    setAgentState("reference_check_wait_fix");
    setWorkflowProgress("reference_check_wait_fix", 2, 4);
    const uint32_t waitStartedMs = millis();
    uint32_t nextSecondMs = waitStartedMs;
    bool fixed = false;
    while (millis() - waitStartedMs < timeoutSeconds * 1000UL) {
        if (workflowCancelRequested_) {
            finish(false, true, "cancelled by user");
            return false;
        }
        if (loopNtripRoverLocked() == 504) {
            finish(false, false, "CORS reference connection lost");
            return false;
        }
        const uint32_t now = millis();
        if (static_cast<int32_t>(now - nextSecondMs) >= 0) {
            nextSecondMs = now + 1000UL;
            const GnssTelemetrySnapshot snapshot = getGnssTelemetrySnapshot();
            fixed = freshRtkSample(now, snapshot.lastGgaMs, snapshot.fixStatus);
            DynamicJsonDocument details(512);
            details["fix_status"] = snapshot.fixStatus;
            details["elapsed"] = (now - waitStartedMs) / 1000UL;
            String serialized;
            serializeJson(details, serialized);
            setWorkflowProgress("reference_check_wait_fix", 2, 4, serialized);
            if (fixed) break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (!fixed) {
        finish(false, false, "RTK fix timeout");
        return false;
    }

    setAgentState("reference_check_averaging");
    setWorkflowProgress("reference_check_averaging", 3, 4);
    std::vector<double> latitudes;
    std::vector<double> longitudes;
    std::vector<double> altitudes;
    latitudes.reserve(requestedSamples);
    longitudes.reserve(requestedSamples);
    altitudes.reserve(requestedSamples);
    const uint32_t averageStartedMs = millis();
    nextSecondMs = averageStartedMs;
    const uint32_t referenceAveragingSeconds = requestedSamples > 60U ? requestedSamples : 60U;
    while (millis() - averageStartedMs < referenceAveragingSeconds * 1000UL
           && latitudes.size() < requestedSamples) {
        if (workflowCancelRequested_) {
            finish(false, true, "cancelled by user");
            return false;
        }
        if (loopNtripRoverLocked() == 504) {
            finish(false, false, "CORS reference connection lost while averaging");
            return false;
        }
        const uint32_t now = millis();
        if (static_cast<int32_t>(now - nextSecondMs) >= 0) {
            nextSecondMs = now + 1000UL;
            const GnssTelemetrySnapshot snapshot = getGnssTelemetrySnapshot();
            const bool usable = freshRtkSample(now, snapshot.lastGgaMs, snapshot.fixStatus);
            if (usable && std::isfinite(snapshot.latitude) && std::isfinite(snapshot.longitude)
                && std::isfinite(snapshot.altitude) && snapshot.latitude >= -90.0
                && snapshot.latitude <= 90.0 && snapshot.longitude >= -180.0
                && snapshot.longitude <= 180.0) {
                latitudes.push_back(snapshot.latitude);
                longitudes.push_back(snapshot.longitude);
                altitudes.push_back(snapshot.altitude);
            }
            DynamicJsonDocument details(768);
            details["samples_collected"] = latitudes.size();
            details["samples_target"] = requestedSamples;
            details["fix_status"] = snapshot.fixStatus;
            String serialized;
            serializeJson(details, serialized);
            setWorkflowProgress("reference_check_averaging", 3, 4, serialized);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (latitudes.size() < 5) {
        finish(false, false, "insufficient valid reference samples");
        return false;
    }

    double measuredLatitude = 0.0;
    double measuredLongitude = 0.0;
    double measuredAltitude = 0.0;
    for (size_t index = 0; index < latitudes.size(); ++index) {
        measuredLatitude += latitudes[index];
        measuredLongitude += longitudes[index];
        measuredAltitude += altitudes[index];
    }
    measuredLatitude /= latitudes.size();
    measuredLongitude /= longitudes.size();
    measuredAltitude /= altitudes.size();
    if (!stopNtripRoverLocked()) {
        finish(false, false, "cannot acquire transport lock to stop CORS reference session");
        return false;
    }

    const double meanLatitudeRad = (measuredLatitude + referenceLatitude)
        * 3.14159265358979323846 / 360.0;
    DynamicJsonDocument result(2048);
    result["reference_coord"]["lat"] = referenceLatitude;
    result["reference_coord"]["lon"] = referenceLongitude;
    result["reference_coord"]["alt"] = referenceAltitude;
    result["measured_coord"]["lat"] = measuredLatitude;
    result["measured_coord"]["lon"] = measuredLongitude;
    result["measured_coord"]["alt"] = measuredAltitude;
    result["delta_n_m"] = (measuredLatitude - referenceLatitude)
        * 3.14159265358979323846 / 180.0 * 6378137.0;
    result["delta_e_m"] = (measuredLongitude - referenceLongitude)
        * 3.14159265358979323846 / 180.0 * 6378137.0 * cos(meanLatitudeRad);
    result["delta_u_m"] = measuredAltitude - referenceAltitude;
    result["sample_count"] = latitudes.size();
    String resultJson;
    serializeJson(result, resultJson);

    if (!cleanup()) {
        setAgentState(normalAgentState());
        setWorkflowProgress("reference_check_failed", 4, 4,
                            "{\"reason\":\"could not safely restore base/NTRIP state\"}");
        saveStatePreferences();
        return false;
    }
    setAgentState(normalAgentState());
    setWorkflowProgress("reference_check_completed", 4, 4, resultJson);
    saveStatePreferences();
    return true;
}
