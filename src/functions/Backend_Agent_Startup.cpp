#include "functions/Backend_Agent.h"
#include "Backend_Agent_Internal.h"
#include "functions/Ublox_Readback.h"
#include "functions/Unicore_Readback.h"
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <cstring>

using namespace BackendAgentInternal;

namespace {
constexpr uint32_t AUTO_SURVEY_IN_SECONDS = 120;

double configNumber(JsonVariantConst value, double fallback) {
    if (value.isNull()) return fallback;
    if (value.is<bool>()) return NAN;
    if (value.is<const char *>()) {
        const char *text = value.as<const char *>();
        char *end = nullptr;
        const double number = std::strtod(text, &end);
        return end != text && *end == '\0' && std::isfinite(number) ? number : double(NAN);
    }
    return value.is<double>() ? value.as<double>() : double(NAN);
}
}

bool BackendAgent::initializeBaseAtBoot(GnssCmdBuilder::GnssSensorType detected) {
    using namespace GnssCmdBuilder;
    if (baseStartupAttempted_) return baseStartupState_ != "error";
    if (detected == GnssSensorType::Invalid) {
        sensorType_ = "UNKNOWN";
        baseStartupState_ = "awaiting_receiver";
        markStatusDirty();
        return false;
    }
    baseStartupAttempted_ = true;
    sensorType_ = detected == GnssSensorType::Unicore ? "Unicorecomm" : "Ublox";
    const auto fail = [&](const String &error) {
        baseStartupState_ = "error";
        baseStartupError_ = error;
        Serial.printf("[GNSS][STARTUP][ERROR] %s\n", error.c_str());
        markStatusDirty();
        return false;
    };
    DynamicJsonDocument config(CONFIG_DOCUMENT_SIZE);
    if (deserializeJson(config, baseConfig_.c_str()) != DeserializationError::Ok || !config.is<JsonObject>())
        return fail("Saved Base configuration is invalid");
    JsonObject base = config.as<JsonObject>();
    const bool saved = base.size() > 0;
    const String method = base["base_setup_method"] | String();
    if (saved && method.isEmpty()) return fail("Saved Base mode is missing");
    if (stationLock_.all) {
        baseStartupState_ = "locked";
        markStatusDirty();
        return true;
    }
    if (method == "AUTO_CORS") {
        baseStartupState_ = "preserved";
        markStatusDirty();
        return true;
    }
    // A mode readback cannot prove that the antenna has not moved.
    // Every Survey-In boot sends a fresh survey configuration.
    bool receiverRead = false;
    String queryError;
    if (detected == GnssSensorType::Unicore) {
        String mode;
        receiverRead = UnicoreReadback::readMode(mode, queryError);
    } else {
        UbxBaseStatus status;
        receiverRead = UbloxReadback::readBaseStatus(status, queryError);
    }
    // Do not guess a mode when a receiver cannot be read reliably.
    if (!receiverRead) return fail(queryError.isEmpty() ? String("Receiver mode could not be read") : queryError);
    const bool restoreFixed = method == "FIXED_LLA";
    if (!restoreFixed && !method.isEmpty() && method != "SURVEY_IN")
        return fail("Unsupported saved Base mode");
    if (!saved) {
        base["base_setup_method"] = "SURVEY_IN";
        base["survey_in_duration"] = AUTO_SURVEY_IN_SECONDS;
        base["survey_in_accuracy"] = 5;
        base["mode"] = "BASE";
        JsonObject options = base.createNestedObject("gnss_options");
        options["output_mode"] = "rtcm_only";
        options["msm_level"] = "both";
    }
    GnssOptions options;
    options.ports = {"UART1"};
    const JsonObjectConst output = base["gnss_options"].as<JsonObjectConst>();
    options.outputMode = String(output["output_mode"] | "rtcm_only") == "rtcm_only"
        ? OutputMode::RtcmOnly : OutputMode::DiagnosticsRtcm;
    const String level = output["msm_level"].isNull() ? "both" : output["msm_level"].as<String>();
    options.msmLevel = level == "7" ? MsmLevel::Msm7 : level == "4" ? MsmLevel::Msm4 : MsmLevel::Both;
    const JsonObjectConst systems = output["constellations"].as<JsonObjectConst>();
    options.gps = systems["gps"] | true; options.glo = systems["glo"] | true;
    options.gal = systems["gal"] | true; options.bds = systems["bds"] | true;
    options.qzss = systems["qzss"] | true; options.sbas = systems["sbas"] | true;
    CommandList commands;
    double fixedLatitude = 0, fixedLongitude = 0, fixedAltitude = 0;
    bool ellipsoid = detected == GnssSensorType::Ublox;
    if (restoreFixed) {
        JsonObjectConst coords = base["coords"].as<JsonObjectConst>();
        if (coords.isNull()) coords = base;
        const double lat = configNumber(coords["lat"], NAN), lon = configNumber(coords["lon"], NAN), alt = configNumber(coords["alt"], NAN);
        const double accuracy = configNumber(base["accuracy"], 0.01);
        if (!std::isfinite(lat) || !std::isfinite(lon) || !std::isfinite(alt)
            || std::abs(lat) > 90 || std::abs(lon) > 180 || alt < -12000 || alt > 100000
            || !std::isfinite(accuracy) || accuracy < 0.0001 || accuracy > 10000)
            return fail("Saved Fixed LLA coordinates or accuracy are invalid");
        fixedLatitude = lat; fixedLongitude = lon; fixedAltitude = alt;
        if (detected == GnssSensorType::Unicore) {
            const String reference = base["altitude_reference"] | "LEGACY";
            if (reference != "ELLIPSOID" && reference != "LEGACY")
                return fail("Unsupported saved altitude reference");
            if (reference == "ELLIPSOID") {
                ellipsoid = true;
                const String zero = "config undulation 0.0\r\n";
                commands.push_back(Command(zero.begin(), zero.end()));
            }
        }
        const CommandList fixed = buildBaseFixedLlaCommand(lat, lon, alt, accuracy, detected, options);
        commands.insert(commands.end(), fixed.begin(), fixed.end());
    } else {
        const double duration = AUTO_SURVEY_IN_SECONDS;
        const double accuracy = configNumber(base["survey_in_accuracy"], 5.0);
        const double maxDuration = detected == GnssSensorType::Unicore ? 3600 : 604800;
        if (!std::isfinite(duration) || std::floor(duration) != duration || duration < 1 || duration > maxDuration
            || !std::isfinite(accuracy) || accuracy < 0.001 || accuracy > 10000)
            return fail("Saved Survey-In parameters are invalid");
        surveyInTargetSeconds_ = duration;
        surveyInMonitoring_ = true;
        surveyInResultPending_ = true;
        commands = buildBaseSurveyInCommand(static_cast<uint32_t>(duration), accuracy, detected, options);
    }
    if (!commands.empty() && !writeGnssCommands(commands)) return fail("Receiver rejected startup configuration");
    if (!saved) {
        baseConfig_.clear(); serializeJson(config, baseConfig_);
    }
    if (!saveStatePreferences()) return fail("Could not save Base configuration");
    baseStartupState_ = restoreFixed ? "fixed_restored"
        : "survey_started";
    lastReceiverPollMs_ = 0;
    String error;
    if (!refreshReceiverReadback(error)) return fail(error);
    DynamicJsonDocument readback(2048);
    deserializeJson(readback, receiverBaseReadback_.c_str());
    const String expected = restoreFixed ? "base" : "survey_in";
    if (String(readback["mode_kind"] | "") != expected) return fail("Receiver mode does not match startup configuration");
    if (restoreFixed) {
        const JsonObjectConst position = readback["bestpos"].as<JsonObjectConst>();
        if (position["state"] != "ok" || position["position_type"] != "FIXEDPOS")
            return fail("Receiver has not confirmed the saved fixed position");
        double lat = 0, lon = 0, height = 0;
        if (detected == GnssSensorType::Ublox) {
            lat = position["lat"].as<double>(); lon = position["lon"].as<double>(); height = position["alt"].as<double>();
        } else {
            const String raw = position["raw"] | String();
            const int separator = raw.indexOf(';');
            double undulation = 0;
            char datum[16] = {};
            if (separator < 0 || std::sscanf(raw.c_str()+separator+1,
                    "%*[^,],%*[^,],%lf,%lf,%lf,%lf,%15[^,]", &lat, &lon, &height, &undulation, datum) != 5
                    || std::strcmp(datum, "WGS84") != 0)
                return fail("Receiver fixed-coordinate readback is invalid");
            if (ellipsoid) height += undulation;
        }
        if (!std::isfinite(lat) || !std::isfinite(lon) || !std::isfinite(height)
            || std::abs(lat-fixedLatitude) > 1e-8 || std::abs(lon-fixedLongitude) > 1e-8
            || std::abs(height-fixedAltitude) > 0.002)
            return fail("Receiver fixed coordinates do not match the saved coordinates");
    }
    Serial.printf("[GNSS][STARTUP] %s; chip=%s\n", baseStartupState_.c_str(), sensorType_.c_str());
    markStatusDirty();
    return true;
}
