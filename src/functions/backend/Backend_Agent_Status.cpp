#include "functions/backend/Backend_Agent.h"
#include "functions/backend/Backend_Agent_Internal.h"
#include "Prog_Config.h"
#include "functions/ntrip/ntrip_base.h"
#include "functions/ntrip/ntrip_rover.h"
#include "functions/ntrip/RTCM_Receiver.h"
#include "functions/backend/Storage_Status.h"
#include <cmath>

#include "functions/mqtt/MQTT_Manager.h"
#include "hardware/Connection_type.h"
#include "hardware/Sim_handler.h"
#include "hardware/Wifi_handler.h"

using namespace BackendAgentInternal;

String BackendAgent::buildLwtPayload() const {
    DynamicJsonDocument doc(1024);
    doc["serial"] = serialNumber_;
    doc["status"] = "offline";
    doc["timestamp"] = epochString().toInt();
    doc["agent_timestamp"] = epochString().toInt();
    doc["status_boot_id"] = statusBootId_;
    doc["status_sequence"] = statusSequence_;
    doc["status_event"] = "lwt";
    String output;
    serializeJson(doc, output);
    return output;
}

String BackendAgent::statusForPayload(bool anyStream, bool onDemand) const
{
    if (stationLock_.all)
        return "locked";
    if (const auto stableState = agentState_ == "configuring" || agentState_ == "rebooting" || agentState_ == "rebooting_for_reset" || agentState_.startsWith("auto_setup") || agentState_.startsWith("reference_check"); stableState)
        return agentState_;
    if (!licenseValid_)
        return "awaiting_license";
    if (!provisioned_)
        return "unprovisioned";
    if (onDemand && !anyStream)
        return "sleep";
    return "online";
}

void BackendAgent::appendStatusFields(JsonDocument &doc, const String &status,
                                      const GnssTelemetrySnapshot &gnss, uint32_t nowMs,
                                      const String &timestamp) const {
    doc["serial"] = serialNumber_;
    doc["name"] = deviceName_;
    doc["status"] = status;
    doc["version"] = AGENT_VERSION;
    doc["agent_token_fingerprint"] = tokenFingerprint(licenseToken_);
    doc["timestamp"] = timestamp.toInt();
    doc["agent_timestamp"] = timestamp.toInt();
    doc["status_boot_id"] = statusBootId_;
    doc["status_sequence"] = statusSequence_;
    doc["rtk_fix_status"] = gnss.fixStatus.data();
    doc["last_raw_gga"] = gnss.lastGga.data();
    doc["last_gga_fix_status"] = gnss.fixStatus.data();
    if (gnss.lastGgaMs != 0) doc["last_raw_gga_age"] = (nowMs - gnss.lastGgaMs) / 1000.0;
    else doc["last_raw_gga_age"] = nullptr;
    doc["base_mode_active"] = baseConfig_ != "{}" && baseConfig_.length() > 2;
    doc["detected_chip_type"] = sensorType_;
    doc["detected_chip_port"] = "Serial1";
    doc["detected_chip_baud"] = Serial1.baudRate();
    doc["is_provisioned"] = provisioned_;
    doc["is_locked"] = stationLock_.all;
    doc["is_synced"] = true;
    doc["is_lte"] = isGsmConnection();
    JsonObject rtcmStats = doc.createNestedObject("rtcm_stats");
    rtcmStats["frames"] = gnss.rtcmFrames;
    rtcmStats["bytes_read"] = gnss.bytesRead;
    rtcmStats["decoder"] = false;
    copyJsonObject(doc, "base_config", baseConfig_);
    copyJsonObject(doc, "service_config", serviceConfig_);
    copyJsonObject(doc, "auto_base_progress", autoBaseProgress_);
}

void BackendAgent::appendAutoBaseProgress(JsonDocument &doc,
                                          const GnssTelemetrySnapshot &gnss) const {
    JsonObject progress = doc["auto_base_progress"].as<JsonObject>();
    if (!progress.isNull() && progress.size() > 0) {
        if (progress["phase"] == "completed" && !(progress["fixed_samples_only"] | false)) {
            progress["phase"] = "unverified";
            progress["reason"] = "Ket qua cu chua duoc xac nhan RTK Fixed; can chay lai Auto Base";
        }
        progress["last_raw_gga"] = gnss.lastGga.data();
        progress["gga_fix_status"] = gnss.fixStatus.data();
        progress["gga_age_seconds"] = doc["last_raw_gga_age"];
        progress["rtcm_bytes"] = ntripRoverBytesReceived();
        progress["rtcm_pkts"] = ntripRoverPacketsReceived();
        JsonObject current = progress.createNestedObject("current_coord");
        current["lat"] = gnss.latitude;
        current["lon"] = gnss.longitude;
        current["alt"] = gnss.altitude;
    }
}

void BackendAgent::appendSystemInfo(JsonDocument &doc, uint32_t nowMs,
                                    const String &timestamp) const {
    JsonObject system = doc.createNestedObject("system_info");
    system["auto_base_review_supported"] = true;
    system["gnss_rf_supported"] = false;
    system["gnss_verification_supported"] = true;
    system["gnss_verification_details_supported"] = true;
    system["gnss_verification_uart1"] = true;
    system["gnss_verification"] = nullptr;
    if (gnssVerification_.length()) {
        DynamicJsonDocument verification(VERIFICATION_DOCUMENT_SIZE);
        if (deserializeJson(verification, gnssVerification_) == DeserializationError::Ok) {
            system["gnss_verification"].set(verification.as<JsonObjectConst>());
            system["gnss_verification"]["age_seconds"] = (millis() - gnssVerificationMs_) / 1000UL;
        }
    }
    appendStartupInfo(system);
    JsonObject survey = appendSurveyInfo(system, nowMs);
    appendReceiverReadback(system, survey);
    appendResourceInfo(system, timestamp);
    JsonObject deviceLock = system.createNestedObject("device_lock");
    deviceLock["scoped"] = true;
    deviceLock["all"] = stationLock_.all;
    JsonArray lockedServers = deviceLock.createNestedArray("locked_servers");
    for (uint8_t id = 1; id <= 2; ++id) {
        if (isServerLocked(id)) lockedServers.add(id);
    }
}

void BackendAgent::appendStartupInfo(JsonObject system) const {
    JsonObject startup = system.createNestedObject("base_startup");
    startup["state"] = baseStartupState_;
    if (!baseStartupError_.isEmpty()) startup["error"] = baseStartupError_;
    if (surveyInResult_.length() > 2) {
        DynamicJsonDocument savedSurvey(1024);
        if (deserializeJson(savedSurvey, surveyInResult_) == DeserializationError::Ok)
            system["survey_in_result"].set(savedSurvey.as<JsonObjectConst>());
    }
}

JsonObject BackendAgent::appendSurveyInfo(JsonObject system, uint32_t nowMs) const {
    JsonObject survey = system.createNestedObject("survey_in_status");
    const bool readbackFresh = receiverBaseReadbackMs_ > 0
        && nowMs - receiverBaseReadbackMs_ <= 45000UL && surveyInReadbackHasPosition_;
    const char *surveyState = "waiting_base";
    if (!surveyInMonitoring_) surveyState = "not_monitoring";
    else if (!readbackFresh) surveyState = "awaiting_readback";
    else if (surveyInReadbackReady_) surveyState = "base_ready";
    survey["state"] = surveyState;
    if (surveyInMonitoring_) {
        if (surveyInStartedMs_ > 0)
            survey["elapsed_since_ack_seconds"] = static_cast<uint32_t>(nowMs - surveyInStartedMs_) / 1000UL;
        else survey["elapsed_since_ack_seconds"] = nullptr;
        survey["target_seconds"] = surveyInTargetSeconds_;
        survey["fix_status"] = surveyInReadbackReady_ ? "FIXEDPOS" : "UNVERIFIED";
        survey["source"] = sensorType_ == "Ublox" ? "NAV-SVIN" : "BESTNAV";
        survey["readback_age_seconds"] = receiverBaseReadbackMs_ > 0
            ? static_cast<uint32_t>(nowMs - receiverBaseReadbackMs_) / 1000UL : UINT32_MAX;
    }
    return survey;
}

void BackendAgent::appendReceiverReadback(JsonObject system, JsonObject survey) const {
    if (receiverBaseReadback_.length() > 2) {
        DynamicJsonDocument readbackDoc(1024);
        if (deserializeJson(readbackDoc, receiverBaseReadback_) == DeserializationError::Ok
            && readbackDoc.is<JsonObject>()) {
            JsonObject readback = system.createNestedObject("receiver_base_readback");
            readback.set(readbackDoc.as<JsonObjectConst>());
            if (surveyInMonitoring_ && sensorType_ == "Ublox" && readbackDoc["state"] == "ok") {
                survey["elapsed_seconds"] = readbackDoc["elapsed_seconds"];
                survey["observations"] = readbackDoc["observations"];
                survey["mean_accuracy_m"] = readbackDoc["mean_accuracy_m"];
                survey["active"] = readbackDoc["active"];
            }
            readback["age_seconds"] = receiverBaseReadbackMs_ > 0
                ? static_cast<uint32_t>(millis() - receiverBaseReadbackMs_) / 1000UL
                : 0;
        }
    }
}

void BackendAgent::appendResourceInfo(JsonObject system, const String &timestamp) const {
    JsonObject cpu = system.createNestedObject("cpu");
    cpu["usage_percent"] = 0.0;
    cpu["frequency_mhz"] = ESP.getCpuFreqMHz();
    cpu["count"] = 2;
    JsonObject memory = system.createNestedObject("memory");
    const uint32_t heapTotal = ESP.getHeapSize();
    const uint32_t heapFree = ESP.getFreeHeap();
    const uint32_t heapUsed = (heapTotal > heapFree) ? (heapTotal - heapFree) : 0;
    memory["free_heap_bytes"] = heapFree;
    memory["total_heap_bytes"] = heapTotal;
    memory["used_bytes"] = heapUsed;
    memory["used_kb"] = heapUsed / 1024;
    memory["total_kb"] = heapTotal / 1024;
    memory["used_mb"] = static_cast<double>(heapUsed) / (1024.0 * 1024.0);
    memory["total_mb"] = static_cast<double>(heapTotal) / (1024.0 * 1024.0);
    memory["percent"] = heapTotal > 0
        ? round((100.0 - (static_cast<double>(heapFree) * 100.0 / heapTotal)) * 10.0) / 10.0
        : 0.0;

    JsonObject disk = system.createNestedObject("disk");
    const uint32_t flashTotal = ESP.getFlashChipSize();
    const uint32_t flashUsed = firmwareSize_;
    const uint32_t flashFree = (flashTotal > flashUsed) ? (flashTotal - flashUsed) : 0;
    disk["type"] = "Flash";
    disk["total_bytes"] = flashTotal;
    disk["free_bytes"] = flashFree;
    disk["used_bytes"] = flashUsed;
    disk["total_mb"] = round((static_cast<double>(flashTotal) / (1024.0 * 1024.0)) * 10.0) / 10.0;
    disk["used_mb"] = round((static_cast<double>(flashUsed) / (1024.0 * 1024.0)) * 10.0) / 10.0;
    disk["total_gb"] = static_cast<double>(flashTotal) / (1024.0 * 1024.0 * 1024.0);
    disk["used_gb"] = static_cast<double>(flashUsed) / (1024.0 * 1024.0 * 1024.0);
    disk["percent"] = flashTotal > 0 ? (round((static_cast<double>(flashUsed) * 1000.0 / flashTotal)) / 10.0) : 0.0;
    appendStorageStatus(system);
    if (nvsInventory_.length() > 2) {
        DynamicJsonDocument inventory(3072);
        if (deserializeJson(inventory, nvsInventory_) == DeserializationError::Ok)
            system["nvs"]["inventory"].set(inventory.as<JsonObjectConst>());
    }
    system["uptime_seconds"] = millis() / 1000UL;
    system["timestamp"] = timestamp.toInt();
    system["auth_license_bypass"] = false;
}

void BackendAgent::appendGnssStatus(JsonDocument &doc, const GnssTelemetrySnapshot &gnss,
                                    uint32_t nowMs) const {
    JsonObject nmeaHealth = doc.createNestedObject("nmea_health");
    JsonObject typeCounts = nmeaHealth.createNestedObject("type_counts");
    typeCounts["GGA"] = gnss.ggaCount;
    typeCounts["GSA"] = gnss.gsaCount;
    typeCounts["GSV"] = gnss.gsvCount;
    typeCounts["GST"] = gnss.gstCount;
    nmeaHealth["gsv_snr_samples"] = gnss.snrSamples;
    if (gnss.lastGsvMs > 0 && nowMs - gnss.lastGsvMs <= 15000UL) {
        nmeaHealth["gsv_snr_samples"] = gnss.latestGsvSamples;
        nmeaHealth["avg_snr"] = gnss.latestAverageSnr;
        nmeaHealth["seconds_since_gsv"] = (nowMs - gnss.lastGsvMs) / 1000.0;
    } else if (gnss.snrSamples > 0) {
        nmeaHealth["avg_snr"] = gnss.averageSnr;
    }
    if (gnss.lastGstMs > 0 && nowMs - gnss.lastGstMs <= 15000UL) {
        nmeaHealth["gst_semi_major_sigma"] = gnss.gstSemiMajor;
        nmeaHealth["gst_semi_minor_sigma"] = gnss.gstSemiMinor;
        nmeaHealth["hacc"] = gnss.gstHacc;
        nmeaHealth["hacc_source"] = "gst_lat_lon_sigma";
        nmeaHealth["seconds_since_gst"] = (nowMs - gnss.lastGstMs) / 1000.0;
    }

    JsonObject position = doc.createNestedObject("position");
    position["latitude"] = gnss.latitude;
    position["longitude"] = gnss.longitude;
    position["altitude"] = gnss.altitude;

    doc["rtk_fix_status"] = gnss.fixStatus.data();
    if (gnss.satellites > 0) doc["satellite_count"] = gnss.satellites;
    doc["hacc"] = nullptr; // Compatibility field: always millimetres, never HDOP.
    doc["gnss_quality"] = nullptr;
    doc["gnss_dop"] = nullptr;
    doc["gnss_satellites"] = nullptr;
    doc["gnss_receiver"] = nullptr;
    if (sensorType_ == "Ublox" && receiverInfoMs_) {
        JsonObject info = doc.createNestedObject("gnss_receiver");
        info["source"] = "ubx_mon_ver";
        info["age_seconds"] = (nowMs - receiverInfoMs_) / 1000.0;
        if (versionReadback_.length()) info["payload_b64"] = versionReadback_;
        if (versionReadbackError_.length()) info["error"] = versionReadbackError_;
    }
    doc["gnss_rf"] = nullptr;
    doc["gnss_spectrum"] = nullptr;
    doc["gnss_rf_monitor"] = nullptr;
    if (sensorType_ == "Ublox" && satelliteReadback_.length()) {
        JsonObject satellites = doc.createNestedObject("gnss_satellites");
        satellites["source"] = "ubx_nav_sat";
        satellites["age_seconds"] = (nowMs - satelliteReadbackMs_) / 1000.0;
        satellites["payload_b64"] = satelliteReadback_;
    }
    if (sensorType_ == "Ublox" && dopMs_ > 0 && nowMs - dopMs_ <= 45000UL) {
        JsonObject dop = doc.createNestedObject("gnss_dop");
        dop["source"] = "ubx_nav_dop";
        dop["age_seconds"] = (nowMs - dopMs_) / 1000.0;
        dop["hdop"] = dop_.hdop * 0.01;
        dop["pdop"] = dop_.pdop * 0.01;
    }
    if (sensorType_ == "Ublox" && navigationMs_ > 0 && nowMs - navigationMs_ <= 45000UL) {
        JsonObject quality = doc.createNestedObject("gnss_quality");
        quality["source"] = "ubx_nav_pvt";
        quality["age_seconds"] = (nowMs - navigationMs_) / 1000.0;
        quality["fix_valid"] = navigation_.valid;
        quality["fix_type"] = navigation_.fixType;
        quality["satellites"] = navigation_.satellites;
        quality["pdop"] = navigation_.pdop * 0.01;
        quality["hacc_m"] = nullptr;
        doc["satellite_count"] = navigation_.satellites;
        if (navigation_.valid) {
            quality["hacc_m"] = navigation_.haccMm * 0.001;
            doc["hacc"] = navigation_.haccMm;
        }
    } else if (sensorType_ == "Unicorecomm" && unicoreQualityMs_ > 0 && nowMs - unicoreQualityMs_ <= 45000UL) {
        JsonObject quality = doc.createNestedObject("gnss_quality");
        quality["source"] = "unicore_bestnav_sigma";
        quality["age_seconds"] = (nowMs - unicoreQualityMs_) / 1000.0;
        quality["hacc_m"] = unicoreHorizontalSigma_;
        doc["hacc"] = unicoreHorizontalSigma_ * 1000.0;
    } else if (gnss.gstHacc > 0 && gnss.lastGstMs > 0
        && nowMs - gnss.lastGstMs <= 15000UL) {
        doc["hacc"] = gnss.gstHacc * 1000.0;
        JsonObject quality = doc.createNestedObject("gnss_quality");
        quality["hacc_m"] = gnss.gstHacc;
        quality["source"] = "nmea_gst";
        quality["age_seconds"] = (nowMs - gnss.lastGstMs) / 1000.0;
    }

    JsonObject gnssStats = doc.createNestedObject("gnss_stats");
    gnssStats["rtcm_packets"] = gnss.rtcmFrames;
    gnssStats["nmea_packets"] = gnss.nmeaSentences;
    gnssStats["bytes_read"] = gnss.bytesRead;
    gnssStats["seconds_since_data"] = gnss.hasData
        ? (nowMs - gnss.lastDataMs) / 1000UL : 0;
    JsonObject parserDebug = gnssStats.createNestedObject("parser_debug");
    parserDebug["rtcm_valid_packets"] = gnss.rtcmFrames;
    parserDebug["rtcm_decoder"] = false;
    parserDebug["nmea_valid_packets"] = gnss.nmeaSentences;
}

void BackendAgent::appendNtripStatus(JsonDocument &doc, const JsonObjectConst &service,
                                     bool server1Active, bool server2Active) const
{
    JsonObject stats = doc.createNestedObject("ntrip_stats");
    stats["rtcm_input_bps"] = getRtcmInputBps();
    stats["server1_bps"] = ntripServerBps(1);
    stats["server2_bps"] = ntripServerBps(2);
    stats["server1_connected"] = ntripServerConnected(1);
    stats["server2_connected"] = ntripServerConnected(2);
    doc["bps"] = ntripServerBps(1) + ntripServerBps(2);
    JsonObject status = doc.createNestedObject("ntrip_status");
    JsonObject server1 = status.createNestedObject("server1");
    server1["connected"] = ntripServerConnected(1);
    server1["error"] = ntripServerError(1);
    server1["bps"] = ntripServerBps(1);
    server1["active"] = server1Active;
    JsonObject server2 = status.createNestedObject("server2");
    server2["connected"] = ntripServerConnected(2);
    server2["error"] = ntripServerError(2);
    server2["bps"] = ntripServerBps(2);
    server2["active"] = server2Active;
    status["rtcm_input_bps"] = getRtcmInputBps();
    doc["ntrip_connected"] = ntripServerConnected(1) || ntripServerConnected(2);
    JsonObject streams = doc.createNestedObject("ntrip_stream_state");
    streams["selected_mountpoint"] = jsonString(service, "active_mountpoint");
    for (uint8_t id = 1; id <= 2; ++id)
    {
        const auto prefix = String("server") + String(id);
        const auto active = id == 1 ? server1Active : server2Active;
        const auto onDemand = jsonBool(service, prefix + "_stream_on_demand",
                                       jsonBool(service, "stream_on_demand", false));
        const auto enabled = jsonBool(service, prefix + "_enabled", false);
        const auto configured = licenseValid_ && provisioned_ && effectiveStreamActive(service, id, false);
        JsonObject stream = streams.createNestedObject(prefix);
        stream["enabled"] = enabled;
        stream["on_demand"] = onDemand;
        stream["active"] = active;
        stream["can_push"] = active;
        stream["configured_can_push"] = configured;
        stream["effective_can_push"] = active;
        stream["locked"] = isServerLocked(id);
        stream["fail_open_forced"] = active && controlPlaneFailOpen_ && !configured;
        stream["sleep"] = enabled && onDemand && !active;
        stream["connected"] = ntripServerConnected(id);
        stream["bps"] = ntripServerBps(id);
        stream["mountpoint"] = jsonString(service, String("mountpoint") + String(id));
    }
}

void BackendAgent::appendTransportStatus(JsonDocument &doc, const GnssTelemetrySnapshot &gnss,
                                         uint32_t nowMs, int32_t signal) const {
    const bool gnssOk = gnss.hasData && nowMs - gnss.lastDataMs <= 15000UL;
    JsonObject system = doc["system_info"];
    JsonObject antenna = doc.createNestedObject("antenna_status");
    antenna["state"] = gnssOk ? "OK" : "FAULT";
    if (!gnssOk) antenna["message"] = "Không nhận được dữ liệu GNSS trong 15 giây";

    const bool mqttConnectedNow = mqttConnected();
    const bool websocketConnectedNow = websocketConnected();
    JsonObject transport = doc.createNestedObject("transport_status");
    transport["mqtt_connected"] = mqttConnectedNow;
    transport["websocket_connected"] = websocketConnectedNow;
    const char *activeChannel = "none";
    if (websocketConnectedNow) activeChannel = "websocket";
    else if (mqttConnectedNow) activeChannel = "mqtt";
    transport["active_channel"] = activeChannel;
    transport["mqtt_broker"] = readPreference("MQTT_SERVER", MQTT_SERVER);
    transport["websocket_host"] = readPreference("BACKEND_HOST", BACKEND_HOST);
    transport["control_plane_fail_open"] = controlPlaneFailOpen_;

    copyJsonObject(doc, "last_command_result", lastCommandResult_);
    JsonObject lastStorageAction = system["nvs"].createNestedObject("last_action");
    const JsonObjectConst lastCommand = doc["last_command_result"];
    lastStorageAction["command_id"] = lastCommand["command_id"];
    lastStorageAction["status"] = lastCommand["status"];
    lastStorageAction["detail"] = lastCommand["detail"];
    JsonObject control = doc.createNestedObject("control_plane_failover");
    control["active"] = controlPlaneFailOpen_;
    control["stale_after_seconds"] = BACKEND_CONTROL_STALE_MS / 1000UL;
    control["recovery_after_seconds"] = BACKEND_CONTROL_RECOVERY_MS / 1000UL;
    JsonObject ackSequences = control.createNestedObject("last_ack_sequence");
    ackSequences["mqtt"] = lastMqttAckSequence_;
    ackSequences["websocket"] = lastWebsocketAckSequence_;

    doc["connected_via"] = isWifiConnection() ? "WIFI" : "GSM";
    doc["rssi_dbm"] = signal;
    doc["free_heap_bytes"] = ESP.getFreeHeap();
    doc["uptime_s"] = millis() / 1000UL;
}

String BackendAgent::buildStatusPayload() {
    if (satelliteReadback_.length() && millis() - satelliteReadbackMs_ > 45000UL)
        satelliteReadback_ = static_cast<const char *>(nullptr);
    if (receiverInfoMs_ && millis() - receiverInfoMs_ > 45000UL) {
        versionReadback_ = versionReadbackError_ = static_cast<const char *>(nullptr);
        receiverInfoMs_ = 0;
    }
    // Inventory is an on-demand snapshot, not permanent telemetry overhead.
    if (nvsInventory_.length() > 2 && millis() - nvsInventoryMs_ >= 60000UL)
        nvsInventory_ = static_cast<const char *>(nullptr);
    if (gnssVerification_.length() && millis() - gnssVerificationMs_ >= 60000UL)
        gnssVerification_ = static_cast<const char *>(nullptr);
    DynamicJsonDocument doc(STATUS_DOCUMENT_SIZE + (receiverInfoMs_ ? versionReadback_.length() + 768 : 0) + (satelliteReadback_.length() ? satelliteReadback_.length() + 256 : 0)
                            + (nvsInventory_.length() > 2 ? 3072 : 0)
                            + (gnssVerification_.length() ? VERIFICATION_DOCUMENT_SIZE : 0));
    DynamicJsonDocument serviceDoc(CONFIG_DOCUMENT_SIZE);
    deserializeJson(serviceDoc, serviceConfig_);
    const JsonObjectConst service = serviceDoc.as<JsonObjectConst>();
    const bool server1Active = licenseValid_ && provisioned_ && !isServerLocked(1)
        && effectiveStreamActive(service, 1, controlPlaneFailOpen_);
    const bool server2Active = licenseValid_ && provisioned_ && !isServerLocked(2)
        && effectiveStreamActive(service, 2, controlPlaneFailOpen_);
    const bool anyStream = server1Active || server2Active;
    const bool onDemand = jsonBool(service, "stream_on_demand", false);

    const String status = statusForPayload(anyStream, onDemand);

    const GnssTelemetrySnapshot gnss = getGnssTelemetrySnapshot();
    const unsigned long nowMs = millis();
    const int32_t signal = isWifiConnection() ? WiFi.RSSI() : modem.getSignalQuality();
    const String timestamp = epochString();

    appendStatusFields(doc, status, gnss, nowMs, timestamp);
    appendAutoBaseProgress(doc, gnss);
    appendSystemInfo(doc, nowMs, timestamp);
    appendGnssStatus(doc, gnss, nowMs);
    appendNtripStatus(doc, service, server1Active, server2Active);
    appendTransportStatus(doc, gnss, nowMs, signal);

    String payload;
    serializeJson(doc, payload);
    return payload;
}
