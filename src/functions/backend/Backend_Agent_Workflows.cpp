#include "functions/backend/Backend_Agent.h"
#include "functions/backend/Backend_Agent_Internal.h"
#include "Prog_Config.h"
#include "functions/ntrip/ntrip_base.h"
#include "functions/ntrip/ntrip_rover.h"
#include "functions/ntrip/RTCM_Receiver.h"
#include "functions/backend/Base_Sample.h"
#include <cmath>
#include <cstring>
#include <vector>
#include <limits>

#include <cstdio>

using namespace BackendAgentInternal;

extern SemaphoreHandle_t tcpStreamMutex;
extern SemaphoreHandle_t rtcmBufferMutex;

namespace {
double coordinateNumber(JsonVariantConst value) {
    if (value.isNull()) return NAN;
    if (value.is<const char *>()) {
        const char *text = value.as<const char *>();
        char *end = nullptr;
        const double number = strtod(text, &end);
        return end != text && *end == '\0' && std::isfinite(number) ? number : NAN;
    }
    return value.is<double>() ? value.as<double>() : NAN;
}

// Keep audit parameters intact across repeated ArduinoJson serialization layers.
String datumParameterText(double value) {
    char text[32];
    snprintf(text, sizeof(text), "%.17g", value);
    return String(text);
}

// Device output quantization is kept separate from all datum calculations.
String receiverCoordinateText(double value, unsigned digits) {
    const double factor = digits == 10 ? 1e10 : digits == 9 ? 1e9 : 1e4;
    double scaled = value * factor;
    const double integer = std::round(scaled);
    // Correct binary representation noise at an exact device-grid boundary.
    if (fabs(scaled - integer) <= 4 * std::numeric_limits<double>::epsilon() * fmax(1.0, fabs(scaled))) scaled = integer;
    return String(std::trunc(scaled) / factor, digits);
}

// Correlation is checked again after crossing the command-thread/workflow queue.
bool acceptAutoBaseLocationReply(JsonObjectConst response, const String &requestId,
                                 double latitude, double longitude, double altitude,
                                 JsonDocument &location) {
    if (response["request_id"].as<String>() != requestId) return false;
    JsonObjectConst result = response["location"].as<JsonObjectConst>();
    JsonObjectConst echoed = result["input_coord"].as<JsonObjectConst>();
    const double lat = coordinateNumber(echoed["lat"]), lon = coordinateNumber(echoed["lon"]), alt = coordinateNumber(echoed["alt"]);
    if (!std::isfinite(lat) || !std::isfinite(lon) || !std::isfinite(alt)
        || fabs(lat - latitude) > 1e-9 || fabs(lon - longitude) > 1e-9 || fabs(alt - altitude) > 0.00011) return false;
    location.set(result);
    return !location.overflowed();
}

}

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
                progress[String(pair.key().c_str())] = pair.value();
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
    auto *agent = static_cast<BackendAgent *>(parameter);
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

bool BackendAgent::restoreWorkflowBase(JsonObjectConst base) {
    using namespace GnssCmdBuilder;
    const auto fail = [&](const String &error) {
        baseStartupState_ = "error";
        baseStartupError_ = error;
        markStatusDirty();
        Serial.printf("[AUTO BASE][RESTORE][ERROR] %s\n", error.c_str());
        return false;
    };
    const String method = base["base_setup_method"] | String();
    JsonObjectConst coords = base["coords"].as<JsonObjectConst>();
    if (coords.isNull()) coords = base;
    const double lat = coordinateNumber(coords["lat"]);
    const double lon = coordinateNumber(coords["lon"]);
    const double alt = coordinateNumber(coords["alt"]);
    if (method.isEmpty() && !std::isfinite(lat) && !std::isfinite(lon) && !std::isfinite(alt)) {
        // An unconfigured receiver may remain Rover for review, but cannot publish a Base stream.
        baseStartupState_ = "awaiting_configuration";
        markStatusDirty();
        return true;
    }
    if (method == "SURVEY_IN") {
        const double duration = base["survey_in_duration"].isNull() ? 120 : coordinateNumber(base["survey_in_duration"]);
        const double accuracy = base["survey_in_accuracy"].isNull() ? 5 : coordinateNumber(base["survey_in_accuracy"]);
        const double maxDuration = sensorType_ == "Unicorecomm" ? 3600 : 604800;
        if (!std::isfinite(duration) || duration < 1 || duration > maxDuration || std::floor(duration) != duration
            || !std::isfinite(accuracy) || accuracy < 0.001 || accuracy > 10000)
            return fail("invalid saved Survey-In parameters");
        GnssOptions options;
        options.ports = {"UART1"};
        const JsonObjectConst output = base["gnss_options"];
        options.outputMode = String(output["output_mode"] | "rtcm_only") == "rtcm_only"
            ? OutputMode::RtcmOnly : OutputMode::DiagnosticsRtcm;
        const String level = output["msm_level"].isNull() ? "both" : output["msm_level"].as<String>();
        options.msmLevel = level == "7" ? MsmLevel::Msm7 : level == "4" ? MsmLevel::Msm4 : MsmLevel::Both;
        const JsonObjectConst systems = output["constellations"];
        options.gps = systems["gps"] | true; options.glo = systems["glo"] | true;
        options.gal = systems["gal"] | true; options.bds = systems["bds"] | true;
        options.qzss = systems["qzss"] | true; options.sbas = systems["sbas"] | true;
        const auto chip = sensorType_ == "Unicorecomm" ? GnssSensorType::Unicore
            : sensorType_ == "Ublox" ? GnssSensorType::Ublox : GnssSensorType::Invalid;
        if (!writeGnssCommands(buildBaseSurveyInCommand(static_cast<uint32_t>(duration), accuracy, chip, options)))
            return fail("receiver rejected Survey-In restore");
        surveyInResultPending_ = true;
    } else if (!std::isfinite(lat) || !std::isfinite(lon) || !std::isfinite(alt)
               || !applyBaseCoordinates(lat, lon, alt)) {
        return fail("cannot restore saved Base coordinates");
    }
    String error;
    if (!refreshReceiverReadback(error)) return fail(error);
    DynamicJsonDocument readback(2048);
    if (deserializeJson(readback, receiverBaseReadback_) != DeserializationError::Ok
        || String(readback["mode_kind"] | "") != (method == "SURVEY_IN" ? "survey_in" : "base"))
        return fail("receiver mode does not match restored Base");
    baseStartupState_ = "configured";
    baseStartupError_.clear();
    markStatusDirty();
    Serial.printf("[AUTO BASE][RESTORE] Previous %s mode confirmed\n", method.c_str());
    return true;
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

bool BackendAgent::waitForAutoBaseFix(uint32_t timeoutSeconds, uint8_t fixedStreakRequired, bool &cancelled, String &failureReason) {
    const uint32_t waitStartedMs = millis();
    uint32_t nextSecondMs = waitStartedMs;
    uint8_t fixedStreak = 0;
    bool fixed = false;
    while (millis() - waitStartedMs < timeoutSeconds * 1000UL) {
        if (workflowCancelRequested_) {
            cancelled = true; failureReason = "cancelled by user";
            return false;
        }
        if (loopNtripRoverLocked() == 504) {
            failureReason = ntripRoverLastError();
            return false;
        }

        const uint32_t now = millis();
        if (static_cast<int32_t>(now - nextSecondMs) >= 0) {
            nextSecondMs = now + 1000UL;
            const GnssTelemetrySnapshot snapshot = getGnssTelemetrySnapshot();
            const bool usable = freshRtkSample(now, snapshot.lastGgaMs, snapshot.fixStatus.data());
            fixedStreak = usable ? static_cast<uint8_t>(fixedStreak + 1) : 0;
            fixed = fixedStreak >= fixedStreakRequired;
            DynamicJsonDocument details(1024);
            details["timeout"] = timeoutSeconds;
            details["elapsed"] = (now - waitStartedMs) / 1000UL;
            details["fix_status"] = snapshot.fixStatus.data();
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
        failureReason = "RTK fix timeout";
        return false;
    }

    return true;
}

bool BackendAgent::collectAutoBaseSamples(int configuredSamples, std::vector<double> &latitudes, std::vector<double> &longitudes, std::vector<double> &altitudes, bool &cancelled, String &failureReason) {
    constexpr uint16_t samplesNeeded = 60;
    constexpr uint32_t averagingSeconds = 60;
    setWorkflowProgress("averaging", 3, 4);
    latitudes.reserve(samplesNeeded);
    longitudes.reserve(samplesNeeded);
    altitudes.reserve(samplesNeeded);

    const uint32_t averageStartedMs = millis();
    uint32_t lastSampleGgaMs = 0;
    uint32_t nextSecondMs = averageStartedMs;
    while (millis() - averageStartedMs < averagingSeconds * 1000UL
           && latitudes.size() < samplesNeeded) {
        if (workflowCancelRequested_) {
            cancelled = true; failureReason = "cancelled by user";
            return false;
        }
        if (loopNtripRoverLocked() == 504) {
            failureReason = ntripRoverLastError();
            return false;
        }

        const uint32_t now = millis();
        if (static_cast<int32_t>(now - nextSecondMs) >= 0) {
            nextSecondMs = now + 1000UL;
            const GnssTelemetrySnapshot snapshot = getGnssTelemetrySnapshot();
            const bool usable = freshRtkSample(now, snapshot.lastGgaMs, snapshot.fixStatus.data());
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
            details["fix_status"] = snapshot.fixStatus.data();
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
        failureReason = "insufficient valid RTK samples";
        return false;
    }

    return true;
}

bool BackendAgent::runAutoBaseWorkflow(JsonObject payload) {
    String host = payload["ip"] | String();
    host.trim();
    const auto port = static_cast<uint16_t>(payload["port"] | 0);
    const String username = payload["user"] | String();
    const String password = payload["password"] | String();
    const String mountpoint = payload["mountpoint"] | String();
    const auto ntripVersion = static_cast<uint8_t>(payload["ntrip_version"] | 1);
    const uint32_t timeoutSeconds = constrain(
        static_cast<uint32_t>(payload["timeout"] | 3600UL), 30UL, 86400UL);
    const auto fixedStreakRequired = static_cast<uint8_t>(constrain(
        static_cast<int>(payload["fixed_streak_seconds"] | 5), 1, 60));

    DynamicJsonDocument previousBase(CONFIG_DOCUMENT_SIZE);
    deserializeJson(previousBase, baseConfig_);
    // Startup can run 120 seconds while the saved UI preference remains 1800.
    // Snapshot the receiver's effective target before Rover readback changes it.
    if (previousBase["base_setup_method"] == "SURVEY_IN")
        previousBase["survey_in_duration"] = surveyInTargetSeconds_;
    auto restorePreviousBase = [&]() {
        return restoreWorkflowBase(previousBase.as<JsonObjectConst>());
    };
    auto finish = [&](const bool success, const bool cancelled, const String &reason) {
        // restartConfiguredNtrip() stops the temporary rover session while
        // holding tcpStreamMutex, then restores the configured source streams.
        const bool roverStopped = stopNtripRoverLocked();
        const bool baseRestored = roverStopped && restorePreviousBase();
        const bool transportRestored = baseRestored && restartConfiguredNtrip();
        const bool cleanupSucceeded = baseRestored && transportRestored;
        setAgentState(normalAgentState());
        DynamicJsonDocument details(1024);
        if (!reason.isEmpty()) details["reason"] = reason;
        if (!transportRestored || !baseRestored) details["cleanup_error"] = "could not restore configured NTRIP transport";
        String serialized;
        serializeJson(details, serialized);
        setWorkflowProgress(
            success && cleanupSucceeded ? "completed" : (cancelled && cleanupSucceeded ? "stopped" : "failed"),
            success && cleanupSucceeded ? 4 : 2, 4, serialized);
        Serial.printf("[AUTO BASE] KET QUA: %s; %s\n",
                      success && cleanupSucceeded ? "THANH CONG"
                          : (cancelled && cleanupSucceeded ? "DA DUNG" : "THAT BAI"), reason.c_str());
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
    bool cancelled = false;
    String failureReason;
    if (!waitForAutoBaseFix(timeoutSeconds, fixedStreakRequired, cancelled, failureReason)) {
        finish(false, cancelled, failureReason);
        return false;
    }

    setAgentState("auto_setup_averaging");
    const int configuredSamples = payload["samples"] | 60;
    std::vector<double> latitudes, longitudes, altitudes;
    if (!collectAutoBaseSamples(configuredSamples, latitudes, longitudes, altitudes, cancelled, failureReason)) {
        finish(false, cancelled, failureReason);
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

    const bool enableTransform = jsonBool(payload, "enable_itrf_vn2000_transform", true);
    setAgentState("auto_setup_base");
    DynamicJsonDocument location(2048);
    Vn2000Datum datum;
    String error = "backend location ACK timed out or did not match sampled LLH";
    bool located = false;
    if (!tcpStreamMutex || xSemaphoreTake(tcpStreamMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS * 4)) != pdTRUE) {
        finish(false, false, "cannot stop rover transport for location lookup"); return false;
    }
    datum = ntripRoverDatum();
    stopNtripRover();
    xSemaphoreGive(tcpStreamMutex);
    if (enableTransform && !datum.has1021) {
        finish(false, false, "missing RTCM 1021 from this CORS session"); return false;
    }
    const bool manualProjection = !payload["l0_deg"].isNull() && !payload["k0"].isNull();
    // Location lookup is only a fallback for missing projection parameters.
    const bool needsLocation = enableTransform && !datum.has1025 && !manualProjection;
    if (needsLocation && autoBaseLocationQueue_) {
        xQueueReset(autoBaseLocationQueue_);
        const String requestId = statusBootId_ + ":" + String(millis());
        StaticJsonDocument<384> requestDoc;
        JsonObject request = requestDoc.createNestedObject("location_request");
        request["request_id"] = requestId;
        request["lat"] = datumParameterText(rawLatitude); request["lon"] = datumParameterText(rawLongitude); request["alt"] = datumParameterText(rawAltitude);
        String requestJson;
        serializeJson(requestDoc, requestJson);
        setWorkflowProgress("resolving_location", 4, 4, requestJson);
        const uint32_t started = millis();
        AutoBaseLocationReply reply;
        DynamicJsonDocument received(2048);
        while (!workflowCancelRequested_ && millis() - started < 30000UL) {
            if (xQueueReceive(autoBaseLocationQueue_, &reply, pdMS_TO_TICKS(50)) == pdTRUE
                // The queue buffer leaves scope before location is used below.
                // Const input makes ArduinoJson own strings copied into location.
                && deserializeJson(received, static_cast<const char *>(reply.json)) == DeserializationError::Ok
                && acceptAutoBaseLocationReply(received.as<JsonObjectConst>(), requestId,
                    rawLatitude, rawLongitude, rawAltitude, location)) { located = true; break; }
        }
    } else if (needsLocation) error = "location reply queue could not be allocated";
    if (workflowCancelRequested_) { finish(false, true, "cancelled by user"); return false; }
    if (needsLocation && !located) {
        location.clear();
        location["state"] = "lookup_failed";
        location["reason"] = error;
        if (enableTransform) { finish(false, false, error); return false; }
    }
    String projectionSource = "not_used";
    Vn2000Result result;
    result.latitude = rawLatitude; result.longitude = rawLongitude; result.altitude = rawAltitude;
    if (enableTransform) {
        if (datum.has1025) projectionSource = "rtcm1025";
        else if (manualProjection) {
            datum.centralMeridianDeg = coordinateNumber(payload["l0_deg"]);
            datum.scale = coordinateNumber(payload["k0"]);
            projectionSource = "manual";
        } else {
            datum.centralMeridianDeg = coordinateNumber(location["l0_deg"]);
            datum.scale = coordinateNumber(location["k0"]);
            if (!std::isfinite(datum.centralMeridianDeg) || !std::isfinite(datum.scale)) {
                finish(false, false, "province projection unresolved; RTCM 1025 or a unique server region is required"); return false;
            }
            projectionSource = "server_catalog";
        }
        for (const char *key : {"resN", "resE", "resHS"}) {
            const double value = payload[key].isNull() ? 0 : coordinateNumber(payload[key]);
            if (!std::isfinite(value)) { finish(false, false, "invalid CORS shift"); return false; }
            if (!strcmp(key, "resN")) datum.resN = value;
            else if (!strcmp(key, "resE")) datum.resE = value;
            else datum.resHS = value;
        }
        if (!transformVn2000(rawLatitude, rawLongitude, rawAltitude, datum, result)) {
            finish(false, false, "VN2000 conversion failed; no candidate coordinates were installed"); return false;
        }
    }
    DynamicJsonDocument progress(CONFIG_DOCUMENT_SIZE);
    JsonObject averaged = progress.createNestedObject("averaged_coord");
    averaged["lat"] = receiverCoordinateText(result.latitude, sensorType_ == "Ublox" ? 9 : 10);
    averaged["lon"] = receiverCoordinateText(result.longitude, sensorType_ == "Ublox" ? 9 : 10);
    averaged["alt"] = sensorType_ == "Ublox" ? receiverCoordinateText(result.altitude, 4) : datumParameterText(result.altitude);
    JsonObject raw = progress.createNestedObject("raw_coord");
    raw["lat"] = datumParameterText(rawLatitude); raw["lon"] = datumParameterText(rawLongitude); raw["alt"] = datumParameterText(rawAltitude);
    JsonObject calculated = progress.createNestedObject("calculated_coord");
    calculated["lat"] = datumParameterText(result.latitude);
    calculated["lon"] = datumParameterText(result.longitude);
    calculated["alt"] = datumParameterText(result.altitude);
    progress["sample_count"] = latitudes.size();
    progress["fixed_samples_only"] = true;
    progress["receiver_write_completed"] = false;
    progress["transform_applied"] = enableTransform;
    JsonObject report = progress.createNestedObject("conversion_report");
    report["version"] = 1;
    report["input_coord"].set(raw);
    report["calculated_coord"].set(calculated);
    report["output_coord"].set(averaged);
    report["source"]["host"] = host; report["source"]["port"] = port; report["source"]["mountpoint"] = mountpoint;
    report["input_reference"] = "receiver_reported";
    report["output_reference"] = enableTransform ? "local_llh" : "receiver_reported";
    report["altitude_reference"] = "ELLIPSOID";
    report["transform_requested"] = enableTransform; report["transform_applied"] = enableTransform;
    report["receiver_write_completed"] = false;
    report["reason"] = enableTransform ? "applied" : "disabled";
    report["sample_count"] = latitudes.size();
    report["location"].set(location.as<JsonObjectConst>());
    report["province_code"] = location["province_code"]; report["province_name"] = location["province_name"];
    report["projection_source"] = projectionSource;
    report["processor"] = "esp32";
    report["zone_width_source"] = "inferred_from_k0";
    Serial.printf("[AUTO BASE][PROJECTION] processor=esp32; source=%s; L0=%.17g; k0=%.17g\n",
                  projectionSource.c_str(), datum.centralMeridianDeg, datum.scale);
    if (datum.has1025) {
        Serial.printf("[AUTO BASE][RTCM1025] LoNO=%lld; SNO=%lu; PT=%u; bytes=%u; hex=%s%s\n",
                      static_cast<long long>(datum.projectionLongitudeRaw),
                      static_cast<unsigned long>(datum.projectionScaleRaw),
                      static_cast<unsigned>(datum.projectionType), static_cast<unsigned>(datum.projectionFrameBytes),
                      datum.projectionFrameHex, datum.projectionFrameBytes > 32 ? " (truncated)" : "");
        JsonObject decoded = report.createNestedObject("rtcm1025");
        decoded["raw_l0"] = String(static_cast<long long>(datum.projectionLongitudeRaw));
        decoded["raw_k0"] = datum.projectionScaleRaw;
        decoded["projection_type"] = datum.projectionType;
        decoded["l0_deg"] = datumParameterText(datum.centralMeridianDeg);
        decoded["k0"] = datumParameterText(datum.scale);
        decoded["false_easting"] = datumParameterText(datum.falseEasting);
        decoded["false_northing"] = datumParameterText(datum.falseNorthing);
        decoded["frame_hex"] = String(datum.projectionFrameHex);
        decoded["frame_bytes"] = datum.projectionFrameBytes;
        decoded["frame_truncated"] = datum.projectionFrameBytes > 32;
    }
    report["parameter_source"] = enableTransform ? "rtcm1021" : "not_used";
    report["residual_source"] = enableTransform && datum.has1023 ? "rtcm1023" : "none";
    report["rtcm1025_used"] = enableTransform && datum.has1025;
    JsonObject counts = report.createNestedObject("received_messages");
    if (datum.count1021) counts["1021"] = datum.count1021;
    if (datum.count1023) counts["1023"] = datum.count1023;
    if (datum.count1025) counts["1025"] = datum.count1025;
    if (enableTransform) {
        if (fabs(datum.scale - 0.9999) < 1e-9) report["zone_width_deg"] = 3;
        else if (fabs(datum.scale - 0.9996) < 1e-9) report["zone_width_deg"] = 6;
        report["central_meridian_deg"] = datumParameterText(datum.centralMeridianDeg);
        report["k0"] = datumParameterText(datum.scale);
        JsonObject projected = report.createNestedObject("projected_coord");
        projected["north"] = datumParameterText(result.north); projected["east"] = datumParameterText(result.east); projected["h"] = datumParameterText(result.h);
        JsonObject parameters = report.createNestedObject("parameters");
        parameters["dX"] = datumParameterText(datum.dX); parameters["dY"] = datumParameterText(datum.dY); parameters["dZ"] = datumParameterText(datum.dZ);
        parameters["rX"] = datumParameterText(datum.rX); parameters["rY"] = datumParameterText(datum.rY); parameters["rZ"] = datumParameterText(datum.rZ);
        parameters["kT"] = datumParameterText(datum.kT);
        parameters["resB"] = datumParameterText(datum.resB); parameters["resL"] = datumParameterText(datum.resL); parameters["resH"] = datumParameterText(datum.resH);
        parameters["resN"] = datumParameterText(datum.resN); parameters["resE"] = datumParameterText(datum.resE); parameters["resHS"] = datumParameterText(datum.resHS);
    }
    if (progress.overflowed()) { finish(false, false, "Auto Base report exceeds memory capacity"); return false; }
    if (!restorePreviousBase() || !restartConfiguredNtrip()) {
        finish(false, false, "cannot restore previous Base/NTRIP state after sampling"); return false;
    }
    setAgentState(normalAgentState());
    String serialized;
    serializeJson(progress, serialized);
    setWorkflowProgress("ready_for_review", 4, 4, serialized);
    saveStatePreferences();
    Serial.printf("[AUTO BASE] Cho duyet; transform=%s; samples=%u; lat=%.17g; lon=%.17g; h=%.17g\n",
        enableTransform ? "VN2000" : "NONE", unsigned(latitudes.size()), result.latitude, result.longitude, result.altitude);
    return true;
}

bool BackendAgent::runReferenceCheckWorkflow(JsonObject payload) {
    JsonObject reference = payload["reference_coords"].as<JsonObject>();
    DynamicJsonDocument storedBase(CONFIG_DOCUMENT_SIZE);
    const bool baseParsed = deserializeJson(storedBase, baseConfig_) == DeserializationError::Ok
        && storedBase.is<JsonObject>();
    JsonObject storedBaseObject = storedBase.as<JsonObject>();
    if (storedBaseObject["base_setup_method"] == "SURVEY_IN")
        storedBaseObject["survey_in_duration"] = surveyInTargetSeconds_;
    JsonObject storedCoordinates = storedBaseObject["coords"].as<JsonObject>();
    if (storedCoordinates.isNull()) storedCoordinates = storedBaseObject;

    const double referenceLatitude = reference["lat"] | NAN;
    const double referenceLongitude = reference["lon"] | NAN;
    const double referenceAltitude = reference["alt"] | NAN;
    const double restoreLatitude = storedCoordinates["lat"] | NAN;
    const double restoreLongitude = storedCoordinates["lon"] | NAN;
    const double restoreAltitude = storedCoordinates["alt"] | NAN;
    const String host = payload["ip"] | String();
    const auto port = static_cast<uint16_t>(payload["port"] | 0);
    const String username = payload["user"] | String();
    const String password = payload["password"] | String();
    const String mountpoint = payload["mountpoint"] | String();
    const uint32_t timeoutSeconds = constrain(
        static_cast<uint32_t>(payload["timeout"] | 300UL), 30UL, 86400UL);
    const auto requestedSamples = static_cast<uint16_t>(constrain(
        static_cast<int>(payload["samples"] | 60), 5, 120));

    auto cleanup = [&]() {
        const bool roverStopped = stopNtripRoverLocked();
        if (!roverStopped) return false;
        const bool baseRestored = restoreWorkflowBase(storedBaseObject);
        const bool transportRestored = baseRestored && restartConfiguredNtrip();
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
            finish(false, false, ntripRoverLastError());
            return false;
        }
        const uint32_t now = millis();
        if (static_cast<int32_t>(now - nextSecondMs) >= 0) {
            nextSecondMs = now + 1000UL;
            const GnssTelemetrySnapshot snapshot = getGnssTelemetrySnapshot();
            fixed = freshRtkSample(now, snapshot.lastGgaMs, snapshot.fixStatus.data());
            DynamicJsonDocument details(512);
            details["fix_status"] = snapshot.fixStatus.data();
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
            const bool usable = freshRtkSample(now, snapshot.lastGgaMs, snapshot.fixStatus.data());
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
            details["fix_status"] = snapshot.fixStatus.data();
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
