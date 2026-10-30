#include "functions/Backend_Agent.h"
#include "Backend_Agent_Internal.h"
#include "functions/Unicore_Readback.h"
#include "functions/Ublox_Readback.h"
#include <cstdio>

using namespace BackendAgentInternal;

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
    if (sensorType_ == "Ublox") {
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
        receiverBaseReadback_.clear(); serializeJson(doc, receiverBaseReadback_);
        receiverBaseReadbackMs_ = millis(); markStatusDirty();
        return success;
    }
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
            surveyInReadbackReady_ = String(bestPosition["position_type"] | "") == "FIXEDPOS";
        }
        saveSurveyResult(bestPosition, mode);

    } else {
        surveyInReadbackHasPosition_ = false;
        readbackDoc["error"] = error;
    }
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
    }

    // MODE/BESTNAV are read-only receiver logs; do not request NMEA output.
}
