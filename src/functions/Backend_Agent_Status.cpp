#include "functions/Backend_Agent.h"
#include "Backend_Agent_Internal.h"
#include "Prog_Config.h"
#include "functions/NTRIP_Handler_IP.h"
#include "functions/RTCM_Receiver.h"
#include <cmath>

#include "functions/MQTT_Manager.h"
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

String BackendAgent::buildStatusPayload() {
    DynamicJsonDocument doc(STATUS_DOCUMENT_SIZE);
    DynamicJsonDocument serviceDoc(CONFIG_DOCUMENT_SIZE);
    deserializeJson(serviceDoc, serviceConfig_);
    const JsonObjectConst service = serviceDoc.as<JsonObjectConst>();
    const bool server1Active = licenseValid_ && provisioned_ && !isServerLocked(1)
        && effectiveStreamActive(service, 1, controlPlaneFailOpen_);
    const bool server2Active = licenseValid_ && provisioned_ && !isServerLocked(2)
        && effectiveStreamActive(service, 2, controlPlaneFailOpen_);
    const bool anyStream = server1Active || server2Active;
    const bool onDemand = jsonBool(service, "stream_on_demand", false);

    String status = agentState_;
    if (stationLock_.all) {
        status = "locked";
    } else if (status != "configuring" && status != "rebooting"
        && status != "rebooting_for_reset" && !status.startsWith("auto_setup")
        && !status.startsWith("reference_check")) {
        if (!licenseValid_) status = "awaiting_license";
        else if (!provisioned_) status = "unprovisioned";
        else if (onDemand && !anyStream) status = "sleep";
        else status = "online";
    }

    const GnssTelemetrySnapshot gnss = getGnssTelemetrySnapshot();
    const unsigned long nowMs = millis();
    const bool gnssOk = gnss.hasData && nowMs - gnss.lastDataMs <= 15000UL;
    const int32_t signal = isWifiConnection() ? WiFi.RSSI() : modem.getSignalQuality();
    const String timestamp = epochString();

    doc["serial"] = serialNumber_;
    doc["name"] = deviceName_;
    doc["status"] = status;
    doc["version"] = AGENT_VERSION;
    doc["agent_token_fingerprint"] = tokenFingerprint(licenseToken_);
    doc["timestamp"] = timestamp.toInt();
    doc["agent_timestamp"] = timestamp.toInt();
    doc["status_boot_id"] = statusBootId_;
    doc["status_sequence"] = statusSequence_;
    doc["rtk_fix_status"] = gnss.fixStatus;
    doc["last_raw_gga"] = gnss.lastGga;
    doc["last_gga_fix_status"] = gnss.fixStatus;
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
    JsonObject progress = doc["auto_base_progress"].as<JsonObject>();
    if (!progress.isNull() && progress.size() > 0) {
        if (progress["phase"] == "completed" && !(progress["fixed_samples_only"] | false)) {
            progress["phase"] = "unverified";
            progress["reason"] = "Ket qua cu chua duoc xac nhan RTK Fixed; can chay lai Auto Base";
        }
        progress["last_raw_gga"] = gnss.lastGga;
        progress["gga_fix_status"] = gnss.fixStatus;
        progress["gga_age_seconds"] = doc["last_raw_gga_age"];
        progress["rtcm_bytes"] = ntripRoverBytesReceived();
        progress["rtcm_pkts"] = ntripRoverPacketsReceived();
        JsonObject current = progress.createNestedObject("current_coord");
        current["lat"] = gnss.latitude;
        current["lon"] = gnss.longitude;
        current["alt"] = gnss.altitude;
    }

    JsonObject system = doc.createNestedObject("system_info");
    JsonObject startup = system.createNestedObject("base_startup");
    startup["state"] = baseStartupState_;
    if (!baseStartupError_.isEmpty()) startup["error"] = baseStartupError_;
    if (surveyInResult_.length() > 2) {
        DynamicJsonDocument savedSurvey(1024);
        if (deserializeJson(savedSurvey, surveyInResult_) == DeserializationError::Ok)
            system["survey_in_result"].set(savedSurvey.as<JsonObjectConst>());
    }
    JsonObject survey = system.createNestedObject("survey_in_status");
    survey["state"] = surveyInMonitoring_
        ? (receiverBaseReadbackMs_ > 0 && nowMs - receiverBaseReadbackMs_ <= 45000UL
           && surveyInReadbackHasPosition_
            ? (surveyInReadbackReady_ ? "base_ready" : "waiting_base") : "awaiting_readback")
        : "not_monitoring";
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
    system["uptime_seconds"] = millis() / 1000UL;
    system["timestamp"] = timestamp.toInt();
    system["auth_license_bypass"] = false;
    JsonObject deviceLock = system.createNestedObject("device_lock");
    deviceLock["scoped"] = true;
    deviceLock["all"] = stationLock_.all;
    JsonArray lockedServers = deviceLock.createNestedArray("locked_servers");
    for (uint8_t id = 1; id <= 2; ++id) {
        if (isServerLocked(id)) lockedServers.add(id);
    }

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

    doc["rtk_fix_status"] = gnss.fixStatus;
    if (gnss.satellites > 0) doc["satellite_count"] = gnss.satellites;
    if (gnss.gstHacc > 0 && gnss.lastGstMs > 0
        && nowMs - gnss.lastGstMs <= 15000UL) {
        doc["hacc"] = gnss.gstHacc;
    } else if (gnss.hdop > 0) {
        doc["hacc"] = gnss.hdop;
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

    JsonObject ntripStats = doc.createNestedObject("ntrip_stats");
    ntripStats["rtcm_input_bps"] = getRtcmInputBps();
    ntripStats["server1_bps"] = ntripServerBps(1);
    ntripStats["server2_bps"] = ntripServerBps(2);
    ntripStats["server1_connected"] = ntripServerConnected(1);
    ntripStats["server2_connected"] = ntripServerConnected(2);
    doc["bps"] = ntripServerBps(1) + ntripServerBps(2);

    JsonObject ntripStatus = doc.createNestedObject("ntrip_status");
    JsonObject server1 = ntripStatus.createNestedObject("server1");
    server1["connected"] = ntripServerConnected(1);
    server1["error"] = ntripServerError(1);
    server1["bps"] = ntripServerBps(1);
    server1["active"] = server1Active;
    JsonObject server2 = ntripStatus.createNestedObject("server2");
    server2["connected"] = ntripServerConnected(2);
    server2["error"] = ntripServerError(2);
    server2["bps"] = ntripServerBps(2);
    server2["active"] = server2Active;
    ntripStatus["rtcm_input_bps"] = getRtcmInputBps();
    doc["ntrip_connected"] = ntripServerConnected(1) || ntripServerConnected(2);

    JsonObject streamState = doc.createNestedObject("ntrip_stream_state");
    streamState["selected_mountpoint"] = jsonString(service, "active_mountpoint");
    JsonObject stream1 = streamState.createNestedObject("server1");
    const bool server1OnDemand = jsonBool(service, "server1_stream_on_demand",
                                          jsonBool(service, "stream_on_demand", false));
    const bool server1Enabled = jsonBool(service, "server1_enabled", false);
    const bool server1Configured = licenseValid_ && provisioned_
        && BackendAgent::effectiveStreamActive(service, 1, false);
    stream1["enabled"] = server1Enabled;
    stream1["on_demand"] = server1OnDemand;
    stream1["active"] = server1Active;
    stream1["can_push"] = server1Active;
    stream1["configured_can_push"] = server1Configured;
    stream1["effective_can_push"] = server1Active;
    stream1["locked"] = isServerLocked(1);
    stream1["fail_open_forced"] = server1Active && controlPlaneFailOpen_ && !server1Configured;
    stream1["sleep"] = server1Enabled && server1OnDemand && !server1Active;
    stream1["connected"] = ntripServerConnected(1);
    stream1["bps"] = ntripServerBps(1);
    stream1["mountpoint"] = jsonString(service, "mountpoint1");
    JsonObject stream2 = streamState.createNestedObject("server2");
    const bool server2OnDemand = jsonBool(service, "server2_stream_on_demand",
                                          jsonBool(service, "stream_on_demand", false));
    const bool server2Enabled = jsonBool(service, "server2_enabled", false);
    const bool server2Configured = licenseValid_ && provisioned_
        && BackendAgent::effectiveStreamActive(service, 2, false);
    stream2["enabled"] = server2Enabled;
    stream2["on_demand"] = server2OnDemand;
    stream2["active"] = server2Active;
    stream2["can_push"] = server2Active;
    stream2["configured_can_push"] = server2Configured;
    stream2["effective_can_push"] = server2Active;
    stream2["locked"] = isServerLocked(2);
    stream2["fail_open_forced"] = server2Active && controlPlaneFailOpen_ && !server2Configured;
    stream2["sleep"] = server2Enabled && server2OnDemand && !server2Active;
    stream2["connected"] = ntripServerConnected(2);
    stream2["bps"] = ntripServerBps(2);
    stream2["mountpoint"] = jsonString(service, "mountpoint2");

    JsonObject antenna = doc.createNestedObject("antenna_status");
    antenna["state"] = gnssOk ? "OK" : "FAULT";
    if (!gnssOk) antenna["message"] = "Không nhận được dữ liệu GNSS trong 15 giây";

    JsonObject transport = doc.createNestedObject("transport_status");
    transport["mqtt_connected"] = mqttConnected();
    transport["websocket_connected"] = websocketConnected();
    transport["active_channel"] = websocketConnected() ? "websocket" : (mqttConnected() ? "mqtt" : "none");
    transport["mqtt_broker"] = readPreference("MQTT_SERVER", MQTT_SERVER);
    transport["websocket_host"] = readPreference("BACKEND_HOST", BACKEND_HOST);
    transport["control_plane_fail_open"] = controlPlaneFailOpen_;

    copyJsonObject(doc, "last_command_result", lastCommandResult_);
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

    String payload;
    serializeJson(doc, payload);
    return payload;
}
