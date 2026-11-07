#include "functions/backend/Backend_Agent.h"
#include "functions/backend/Backend_Agent_Internal.h"
#include "Prog_Config.h"
#include "functions/ntrip/ntrip_base.h"
#include "functions/ntrip/ntrip_rover.h"
#include "functions/ntrip/RTCM_Receiver.h"
#include <cmath>

#include "functions/mqtt/MQTT_Manager.h"
#include "hardware/Connection_type.h"
#include "hardware/Sim_handler.h"
#include "hardware/Wifi_handler.h"

using namespace BackendAgentInternal;

String BackendAgent::buildLwtPayload() const
{
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
                                      const String &timestamp) const
{
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
    if (gnss.lastGgaMs != 0)
        doc["last_raw_gga_age"] = (nowMs - gnss.lastGgaMs) / 1000.0;
    else
        doc["last_raw_gga_age"] = nullptr;
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
                                          const GnssTelemetrySnapshot &gnss,
                                          [[maybe_unused]] uint32_t nowMs) const
{
    JsonObject progress = doc["auto_base_progress"].as<JsonObject>();
    if (progress.isNull() || progress.size() == 0)
        return;
    if (progress["phase"] == "completed" && !(progress["fixed_samples_only"] | false))
    {
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

void BackendAgent::appendSystemInfo(JsonDocument &doc, uint32_t nowMs,
                                    const String &timestamp) const
{
    JsonObject system = doc.createNestedObject("system_info");
    appendStartupInfo(system);
    JsonObject survey = appendSurveyInfo(system, nowMs);
    appendReceiverReadback(system, survey);
    appendResourceInfo(system, timestamp);
    JsonObject deviceLock = system.createNestedObject("device_lock");
    deviceLock["scoped"] = true;
    deviceLock["all"] = stationLock_.all;
    JsonArray lockedServers = deviceLock.createNestedArray("locked_servers");
    for (uint8_t id = 1; id <= 2; ++id)
        if (isServerLocked(id))
            lockedServers.add(id);
}

void BackendAgent::appendStartupInfo(JsonObject system) const
{
    JsonObject startup = system.createNestedObject("base_startup");
    startup["state"] = baseStartupState_;
    if (!baseStartupError_.isEmpty())
        startup["error"] = baseStartupError_;
    if (surveyInResult_.length() > 2)
    {
        DynamicJsonDocument savedSurvey(1024);
        if (deserializeJson(savedSurvey, surveyInResult_) == DeserializationError::Ok)
            system["survey_in_result"].set(savedSurvey.as<JsonObjectConst>());
    }
}

JsonObject BackendAgent::appendSurveyInfo(JsonObject system, uint32_t nowMs) const
{
    JsonObject survey = system.createNestedObject("survey_in_status");
    const auto readbackFresh = receiverBaseReadbackMs_ > 0 && nowMs - receiverBaseReadbackMs_ <= 45000UL && surveyInReadbackHasPosition_;
    const char *surveyState = "waiting_base";
    if (!surveyInMonitoring_)
        surveyState = "not_monitoring";
    else if (!readbackFresh)
        surveyState = "awaiting_readback";
    else if (surveyInReadbackReady_)
        surveyState = "base_ready";
    survey["state"] = surveyState;
    if (surveyInMonitoring_)
    {
        if (surveyInStartedMs_ > 0)
            survey["elapsed_since_ack_seconds"] =
                (nowMs - surveyInStartedMs_) / 1000UL;
        else
            survey["elapsed_since_ack_seconds"] = nullptr;
        survey["target_seconds"] = surveyInTargetSeconds_;
        survey["fix_status"] = surveyInReadbackReady_ ? "FIXEDPOS" : "UNVERIFIED";
        survey["source"] = sensorType_ == "Ublox" ? "NAV-SVIN" : "BESTNAV";
        survey["readback_age_seconds"] = receiverBaseReadbackMs_ > 0
                                             ? (nowMs - receiverBaseReadbackMs_) / 1000UL
                                             : UINT32_MAX;
    }
    return survey;
}

void BackendAgent::appendReceiverReadback(JsonObject system, JsonObject survey) const
{
    if (receiverBaseReadback_.length() > 2)
    {
        DynamicJsonDocument readbackDoc(1024);
        if (deserializeJson(readbackDoc, receiverBaseReadback_) == DeserializationError::Ok && readbackDoc.is<JsonObject>())
        {
            JsonObject readback = system.createNestedObject("receiver_base_readback");
            readback.set(readbackDoc.as<JsonObjectConst>());
            if (surveyInMonitoring_ && sensorType_ == "Ublox" && readbackDoc["state"] == "ok")
            {
                survey["elapsed_seconds"] = readbackDoc["elapsed_seconds"];
                survey["observations"] = readbackDoc["observations"];
                survey["mean_accuracy_m"] = readbackDoc["mean_accuracy_m"];
                survey["active"] = readbackDoc["active"];
            }
            readback["age_seconds"] = receiverBaseReadbackMs_ > 0
                                           ? (millis() - receiverBaseReadbackMs_) / 1000UL
                                           : 0;
        }
    }
}

void BackendAgent::appendResourceInfo(JsonObject system, const String &timestamp) const
{
    JsonObject cpu = system.createNestedObject("cpu");
    cpu["usage_percent"] = 0.0;
    cpu["frequency_mhz"] = ESP.getCpuFreqMHz();
    cpu["count"] = 2;
    JsonObject memory = system.createNestedObject("memory");
    const uint32_t heapTotal = ESP.getHeapSize();
    const uint32_t heapFree = ESP.getFreeHeap();
    const uint32_t heapUsed = heapTotal > heapFree ? heapTotal - heapFree : 0;
    memory["free_heap_bytes"] = heapFree;
    memory["total_heap_bytes"] = heapTotal;
    memory["used_bytes"] = heapUsed;
    memory["used_kb"] = heapUsed / 1024;
    memory["total_kb"] = heapTotal / 1024;
    memory["used_mb"] = static_cast<double>(heapUsed) / (1024.0 * 1024.0);
    memory["total_mb"] = static_cast<double>(heapTotal) / (1024.0 * 1024.0);
    memory["percent"] = heapTotal > 0
                            ? round((100.0 - static_cast<double>(heapFree) * 100.0 / heapTotal) * 10.0) / 10.0
                            : 0.0;
    JsonObject disk = system.createNestedObject("disk");
    const uint32_t flashTotal = ESP.getFlashChipSize();
    const uint32_t flashUsed = firmwareSize_;
    const uint32_t flashFree = flashTotal > flashUsed ? flashTotal - flashUsed : 0;
    disk["type"] = "Flash";
    disk["total_bytes"] = flashTotal;
    disk["free_bytes"] = flashFree;
    disk["used_bytes"] = flashUsed;
    disk["total_mb"] = round(static_cast<double>(flashTotal) / (1024.0 * 1024.0) * 10.0) / 10.0;
    disk["used_mb"] = round(static_cast<double>(flashUsed) / (1024.0 * 1024.0) * 10.0) / 10.0;
    disk["total_gb"] = static_cast<double>(flashTotal) / (1024.0 * 1024.0 * 1024.0);
    disk["used_gb"] = static_cast<double>(flashUsed) / (1024.0 * 1024.0 * 1024.0);
    disk["percent"] = flashTotal > 0 ? round(static_cast<double>(flashUsed) * 1000.0 / flashTotal) / 10.0 : 0.0;
    system["uptime_seconds"] = millis() / 1000UL;
    system["timestamp"] = timestamp.toInt();
    system["auth_license_bypass"] = false;
}

void BackendAgent::appendGnssStatus(JsonDocument &doc, const GnssTelemetrySnapshot &gnss,
                                    uint32_t nowMs) const
{
    JsonObject nmeaHealth = doc.createNestedObject("nmea_health");
    JsonObject typeCounts = nmeaHealth.createNestedObject("type_counts");
    typeCounts["GGA"] = gnss.ggaCount;
    typeCounts["GSA"] = gnss.gsaCount;
    typeCounts["GSV"] = gnss.gsvCount;
    typeCounts["GST"] = gnss.gstCount;
    nmeaHealth["gsv_snr_samples"] = gnss.snrSamples;
    if (gnss.lastGsvMs > 0 && nowMs - gnss.lastGsvMs <= 15000UL)
    {
        nmeaHealth["gsv_snr_samples"] = gnss.latestGsvSamples;
        nmeaHealth["avg_snr"] = gnss.latestAverageSnr;
        nmeaHealth["seconds_since_gsv"] = (nowMs - gnss.lastGsvMs) / 1000.0;
    }
    else if (gnss.snrSamples > 0)
        nmeaHealth["avg_snr"] = gnss.averageSnr;
    if (gnss.lastGstMs > 0 && nowMs - gnss.lastGstMs <= 15000UL)
    {
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
    if (gnss.satellites > 0)
        doc["satellite_count"] = gnss.satellites;
    if (gnss.gstHacc > 0 && gnss.lastGstMs > 0 && nowMs - gnss.lastGstMs <= 15000UL)
        doc["hacc"] = gnss.gstHacc;
    else if (gnss.hdop > 0)
        doc["hacc"] = gnss.hdop;
    JsonObject stats = doc.createNestedObject("gnss_stats");
    stats["rtcm_packets"] = gnss.rtcmFrames;
    stats["nmea_packets"] = gnss.nmeaSentences;
    stats["bytes_read"] = gnss.bytesRead;
    stats["seconds_since_data"] = gnss.hasData ? (nowMs - gnss.lastDataMs) / 1000UL : 0;
    JsonObject parser = stats.createNestedObject("parser_debug");
    parser["rtcm_valid_packets"] = gnss.rtcmFrames;
    parser["rtcm_decoder"] = false;
    parser["nmea_valid_packets"] = gnss.nmeaSentences;
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

void BackendAgent::appendTransportStatus(JsonDocument &doc,
                                         const GnssTelemetrySnapshot &gnss,
                                         uint32_t nowMs, int32_t signal) const
{
    const auto mqtt_connected = mqttConnected();
    const auto websocket = websocketConnected();
    JsonObject antenna = doc.createNestedObject("antenna_status");
    const auto gnssOk = gnss.hasData && nowMs - gnss.lastDataMs <= 15000UL;
    antenna["state"] = gnssOk ? "OK" : "FAULT";
    if (!gnssOk)
        antenna["message"] = "Không nhận được dữ liệu GNSS trong 15 giây";
    JsonObject transport = doc.createNestedObject("transport_status");
    transport["mqtt_connected"] = mqtt_connected;
    transport["websocket_connected"] = websocket;
    const char *activeChannel = "none";
    if (websocket)
        activeChannel = "websocket";
    else if (mqtt_connected)
        activeChannel = "mqtt";
    transport["active_channel"] = activeChannel;
    transport["mqtt_broker"] = readPreference("MQTT_SERVER", MQTT_SERVER);
    transport["websocket_host"] = readPreference("BACKEND_HOST", BACKEND_HOST);
    transport["control_plane_fail_open"] = controlPlaneFailOpen_;
    copyJsonObject(doc, "last_command_result", lastCommandResult_);
    JsonObject control = doc.createNestedObject("control_plane_failover");
    control["active"] = controlPlaneFailOpen_;
    control["stale_after_seconds"] = BACKEND_CONTROL_STALE_MS / 1000UL;
    control["recovery_after_seconds"] = BACKEND_CONTROL_RECOVERY_MS / 1000UL;
    JsonObject acks = control.createNestedObject("last_ack_sequence");
    acks["mqtt"] = lastMqttAckSequence_;
    acks["websocket"] = lastWebsocketAckSequence_;
    doc["connected_via"] = isWifiConnection() ? "WIFI" : "GSM";
    doc["rssi_dbm"] = signal;
    doc["free_heap_bytes"] = ESP.getFreeHeap();
    doc["uptime_s"] = millis() / 1000UL;
}

String BackendAgent::buildStatusPayload()
{
    DynamicJsonDocument doc(STATUS_DOCUMENT_SIZE);
    DynamicJsonDocument serviceDoc(CONFIG_DOCUMENT_SIZE);
    deserializeJson(serviceDoc, serviceConfig_);
    const JsonObjectConst service = serviceDoc.as<JsonObjectConst>();
    const auto server1Active = licenseValid_ && provisioned_ && !isServerLocked(1) && effectiveStreamActive(service, 1, controlPlaneFailOpen_);
    const auto server2Active = licenseValid_ && provisioned_ && !isServerLocked(2) && effectiveStreamActive(service, 2, controlPlaneFailOpen_);
    const auto onDemand = jsonBool(service, "stream_on_demand", false);
    const GnssTelemetrySnapshot gnss = getGnssTelemetrySnapshot();
    const uint32_t nowMs = millis();
    const int32_t signal = isWifiConnection() ? WiFi.RSSI() : modem.getSignalQuality();
    const auto timestamp = epochString();
    appendStatusFields(doc, statusForPayload(server1Active || server2Active, onDemand),
                       gnss, nowMs, timestamp);
    appendAutoBaseProgress(doc, gnss, nowMs);
    appendSystemInfo(doc, nowMs, timestamp);
    appendGnssStatus(doc, gnss, nowMs);
    appendNtripStatus(doc, service, server1Active, server2Active);
    appendTransportStatus(doc, gnss, nowMs, signal);
    String payload;
    serializeJson(doc, payload);
    return payload;
}
