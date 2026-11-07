#include "functions/backend/Backend_Agent.h"
#include "functions/backend/Backend_Agent_Internal.h"
#include "Prog_Config.h"
#include "functions/ntrip/ntrip_base.h"
#include "functions/ntrip/RTCM_Receiver.h"
#include <array>
#include <cmath>
#include <cstring>

#include "functions/mqtt/MQTT_Manager.h"
#include "functions/gnss/Unicore_Log.h"
#include "functions/gnss/Unicore_Readback.h"

using namespace BackendAgentInternal;

extern SemaphoreHandle_t rtcmBufferMutex;

namespace
{

    String normalizedStreamValue(String value, bool stripLeadingSlash)
    {
        value.trim();
        if (stripLeadingSlash)
        {
            while (value.startsWith("/"))
                value.remove(0, 1);
        }
        value.toLowerCase();
        return value;
    }

    int streamServerId(JsonObjectConst payload)
    {
        auto serverId = payload["server_id"] | 0;
        if (serverId != 1 && serverId != 2)
            serverId = payload["server_idx"] | 0;
        if (serverId != 1 && serverId != 2)
            serverId = payload["server"] | 0;
        return serverId;
    }

    void setStreamServerState(JsonObject service, uint8_t id, bool active,
                              const String &mountpoint)
    {
        const auto prefix = String("server") + String(id);
        const auto onDemand = jsonBool(
            service, prefix + "_stream_on_demand",
            jsonBool(service, "stream_on_demand", false));
        if (!active && !onDemand)
            return;
        service[(prefix + "_stream_active").c_str()] = active;
        if (active && !mountpoint.isEmpty())
        {
            service["active_mountpoint"] = mountpoint;
            return;
        }
        if (!active)
        {
            const auto configured = jsonString(service, String("mountpoint") + String(id));
            const auto current = jsonString(service, "active_mountpoint");
            if (mountpoint.isEmpty() || mountpoint.equalsIgnoreCase(configured) || current.equalsIgnoreCase(configured))
            {
                service.remove("active_mountpoint");
            }
        }
    }

    bool serverMatchesAddress(JsonObjectConst service, uint8_t id, const String &address)
    {
        const auto prefix = String("server") + String(id);
        auto configured = jsonString(service, prefix + "_host",
                                     jsonString(service, String("serverhost") + String(id)));
        configured = normalizedStreamValue(configured, false);
        return !configured.isEmpty() && (address.indexOf(configured) >= 0 || configured.indexOf(address) >= 0);
    }

    bool serverMatchesMountpoint(JsonObjectConst service, uint8_t id,
                                 const String &mountpoint)
    {
        auto configured = normalizedStreamValue(
            jsonString(service, String("mountpoint") + String(id)), true);
        return !configured.isEmpty() && configured == mountpoint;
    }

    bool applyTargetedStreamState(JsonObject service, int serverId, bool active,
                                  const String &address, const String &mountpoint)
    {
        if (serverId == 1 || serverId == 2)
        {
            setStreamServerState(service, static_cast<uint8_t>(serverId), active, mountpoint);
            return true;
        }
        if (!address.isEmpty())
        {
            auto targeted = false;
            for (uint8_t id = 1; id <= 2; ++id)
            {
                if (!serverMatchesAddress(service, id, address))
                    continue;
                setStreamServerState(service, id, active, mountpoint);
                targeted = true;
            }
            return targeted;
        }
        if (!mountpoint.isEmpty())
        {
            auto targeted = false;
            for (uint8_t id = 1; id <= 2; ++id)
            {
                if (!serverMatchesMountpoint(service, id, mountpoint))
                    continue;
                setStreamServerState(service, id, active, mountpoint);
                targeted = true;
            }
            return targeted;
        }
        if (active)
            return false;
        for (uint8_t id = 1; id <= 2; ++id)
            setStreamServerState(service, id, active, mountpoint);
        return true;
    }

    void applyGlobalStreamState(JsonObject service, bool active, const String &mountpoint)
    {
        service["stream_active"] = active;
        if (!mountpoint.isEmpty() && active)
            service["active_mountpoint"] = mountpoint;
        if (!active)
            service.remove("active_mountpoint");
    }

}

void BackendAgent::recordCommandResult(const String &command, const String &source,
                                       const String &status, const String &detail,
                                       const String &commandId)
{
    DynamicJsonDocument doc(1024);
    doc["command"] = command;
    doc["source"] = source;
    doc["status"] = status;
    doc["detail"] = detail.substring(0, 300);
    doc["command_id"] = commandId;
    doc["timestamp"] = epochString().toInt();
    lastCommandResult_.clear();
    serializeJson(doc, lastCommandResult_);
    const auto saved = saveStatePreferences();
    Serial.printf("[BACKEND][CMD] %s id=%s source=%s status=%s; NVS=%s\n",
                  command.c_str(), commandId.c_str(), source.c_str(), status.c_str(),
                  saved ? "OK" : "LOI");
}

void BackendAgent::publishConfigState(const char *kind)
{
    if (!mqttConnected())
        return;
    DynamicJsonDocument doc(CONFIG_DOCUMENT_SIZE);
    doc["serial"] = serialNumber_;
    doc["timestamp"] = epochString().toInt();
    doc["type"] = kind;
    if (strcmp(kind, "base_config_state") == 0)
    {
        copyJsonObject(doc, "config", baseConfig_);
    }
    else
    {
        copyJsonObject(doc, "config", serviceConfig_);
    }
    String output;
    serializeJson(doc, output);
    const auto topic = String("pi/devices/") + serialNumber_ + "/" + kind;
    mqtt.publish(topic.c_str(), output.c_str(), true);
}

void BackendAgent::applyServiceStreamCommand(JsonObject payload)
{
    DynamicJsonDocument serviceDoc(CONFIG_DOCUMENT_SIZE);
    if (deserializeJson(serviceDoc, serviceConfig_) != DeserializationError::Ok || !serviceDoc.is<JsonObject>())
    {
        serviceDoc.to<JsonObject>();
    }
    JsonObject service = serviceDoc.as<JsonObject>();
    const auto active = payload["active"] | false;
    auto mountpoint = normalizedStreamValue(payload["mountpoint"] | String(), true);
    auto address = payload["address"] | String();
    if (address.isEmpty())
        address = payload["caster_host"] | String();
    if (address.isEmpty())
        address = payload["ip"] | String();
    if (address.isEmpty())
        address = payload["host"] | String();
    address = normalizedStreamValue(address, false);
    const auto serverId = streamServerId(payload);

    if (!applyTargetedStreamState(service, serverId, active, address, mountpoint))
        applyGlobalStreamState(service, active, mountpoint);
    service["stream_active"] = BackendAgent::effectiveStreamActive(service, 1, false) || BackendAgent::effectiveStreamActive(service, 2, false);
    serviceConfig_.clear();
    serializeJson(serviceDoc, serviceConfig_);
    saveStatePreferences();
}

bool BackendAgent::applyLockCommand(JsonObjectConst payload, String &error)
{
    if (!payload.containsKey("locked_servers"))
    {
        requestWorkflowStop();
        stationLock_.all = true;
        stationLock_.servers = 0;
        return true;
    }

    const JsonArrayConst ids = payload["locked_servers"].as<JsonArrayConst>();
    if (ids.isNull())
    {
        error = "payload.locked_servers must contain only server IDs 1 or 2";
        return false;
    }

    int serverIds[2] = {};
    auto count = 0;
    uint8_t seen = 0;
    for (JsonVariantConst id : ids)
    {
        if (!id.is<int>() || id.as<int>() < 1 || id.as<int>() > 2)
        {
            error = "payload.locked_servers must contain only server IDs 1 or 2";
            return false;
        }
        const auto serverId = id.as<int>();
        const auto bit = static_cast<uint8_t>(1U << (serverId - 1));
        if ((seen & bit) == 0 && count < 2)
        {
            serverIds[count++] = serverId;
            seen = static_cast<uint8_t>(seen | bit);
        }
    }
    if (!stationLock_.setScoped(serverIds, count))
    {
        error = "payload.locked_servers must contain only server IDs 1 or 2";
        return false;
    }
    return true;
}

bool BackendAgent::executeRawCommands(JsonArray commands, String &error, const String &sensorType)
{
    if (commands.isNull() || commands.size() == 0)
    {
        error = "missing payload.commands_b64";
        return false;
    }
    if (commands.size() > 200)
    {
        error = "too many raw commands";
        return false;
    }
    uint8_t decoded[3072] = {};
    for (JsonVariant item : commands)
    {
        const auto encoded = item.as<String>();
        if (encoded.isEmpty() || encoded.length() > 4096)
        {
            error = "invalid raw command item";
            return false;
        }
        auto length = decodeBase64(encoded, decoded, sizeof(decoded));
        if (length == 0)
        {
            error = "raw command is not valid base64";
            return false;
        }
        uint32_t delayMs = 0;
        if (delayMarker(decoded, length, delayMs))
        {
            delay(delayMs);
            continue;
        }
        if (sensorType == "Unicorecomm")
            length = routeUnicoreOutput(decoded, length, GNSS_UNICORE_PORT);
        else if (sensorType == "Ublox")
            GnssCmdBuilder::routeUbloxOutput(decoded, length);
        const auto unicore = sensorType == "Unicorecomm" || (sensorType.isEmpty() && sensorType_ == "Unicorecomm");
        if (unicore)
        {
            if (!writeUnicoreCommand(decoded, length, error))
                return false;
            continue;
        }
        if (rtcmBufferMutex && xSemaphoreTake(rtcmBufferMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE)
        {
            Serial1.write(decoded, length);
            Serial1.flush();
            xSemaphoreGive(rtcmBufferMutex);
        }
        else
        {
            error = "GNSS serial is busy";
            return false;
        }
        delay(50);
    }
    return true;
}

void BackendAgent::handleCommand(const String &source, const String &message)
{
    Serial.printf("[BACKEND][RX] Nhan goi lenh qua %s (%u bytes).\n",
                  source.c_str(), static_cast<unsigned>(message.length()));
    DynamicJsonDocument doc(COMMAND_DOCUMENT_SIZE);
    if (deserializeJson(doc, message) != DeserializationError::Ok || !doc.is<JsonObject>())
    {
        recordCommandResult("UNKNOWN", source, "rejected", "command envelope must be an object", "");
        publishStatus(true);
        return;
    }

    const auto command = doc["command"] | String();
    const JsonObject payload = doc["payload"].as<JsonObject>();
    auto commandId = doc["command_id"] | String();
    if (commandId.isEmpty() && !payload.isNull())
        commandId = payload["command_id"] | String();
    if (commandId.isEmpty())
        commandId = String("esp32-") + String(millis());
    Serial.printf("[BACKEND][CMD] Nhan command=%s id=%s qua %s.\n",
                  command.c_str(), commandId.c_str(), source.c_str());
    if (command.isEmpty())
    {
        recordCommandResult("UNKNOWN", source, "rejected", "missing command", commandId);
        publishStatus(true);
        return;
    }

    const auto allowed = command == "LOCK_DEVICE" || command == "UNLOCK_DEVICE" || command == "PROVISION_DEVICE" || command == "DEPLOY_LICENSE" || command == "TRIGGER_AUTO_BASE" || command == "TRIGGER_BASE_REFERENCE_CHECK" || command == "STOP_AUTO_BASE" || command == "APPLY_VN2000_PROVINCE" || command == "EXECUTE_RAW_COMMANDS" || command == "DEPLOY_SERVICE_CONFIG" || command == "SET_RTCM_STREAM_ACTIVE" || command == "DELETE_DEVICE" || command == "CHECK_BASE_STATUS" || command == "REBOOT_DEVICE";
    if (!allowed)
    {
        recordCommandResult(command, source, "rejected", "unsupported command", commandId);
        publishStatus(true);
        return;
    }
    if (stationLock_.all && command != "LOCK_DEVICE" && command != "UNLOCK_DEVICE")
    {
        recordCommandResult(command, source, "rejected", "device is locked", commandId);
        publishStatus(true);
        return;
    }
    if (!licenseValid_ && command != "LOCK_DEVICE" && command != "UNLOCK_DEVICE" && command != "PROVISION_DEVICE" && command != "DEPLOY_LICENSE")
    {
        recordCommandResult(command, source, "rejected", "license is missing or invalid", commandId);
        publishStatus(true);
        return;
    }

    recordCommandResult(command, source, "running", "", commandId);
    executeCommand(source, command, commandId, doc, payload);
}

bool BackendAgent::executeLicenseCommand(JsonObject payload, String &error,
                                         String &detail, bool &restartAfterCommand)
{
    const auto license = payload["license_key"] | String();
    if (license.isEmpty())
    {
        Serial.println("[BACKEND][LICENSE] Lenh DEPLOY_LICENSE khong co license_key.");
        error = "missing payload.license_key";
        return false;
    }
    if (!licenseMatchesSerial(serialNumber_, license))
    {
        error = "invalid device token or legacy license";
        return false;
    }
    Serial.println("[BACKEND][LICENSE] Da nhan license_key (khong in noi dung key).");
    const auto previousToken = licenseToken_;
    const auto previousValid = licenseValid_;
    licenseToken_ = license;
    licenseValid_ = true;
    const auto saved = saveStatePreferences();
    Serial.printf("[BACKEND][LICENSE] Kiem tra: %s; luu NVS: %s.\n",
                  licenseValid_ ? "HOP LE" : "KHONG HOP LE",
                  saved ? "THANH CONG" : "THAT BAI");
    if (!saved)
    {
        licenseToken_ = previousToken;
        licenseValid_ = previousValid;
        error = "cannot persist device token";
        return false;
    }
    detail = "license deployed; rebooting";
    restartAfterCommand = true;
    setAgentState("rebooting");
    refreshWebsocket();
    return true;
}

bool BackendAgent::executeServiceConfigCommand(JsonObject payload, String &error,
                                               String &detail)
{
    if (payload.isNull() || payload.size() == 0)
    {
        error = "missing service config payload";
        return false;
    }
    DynamicJsonDocument serviceDoc(CONFIG_DOCUMENT_SIZE);
    if (deserializeJson(serviceDoc, serviceConfig_) != DeserializationError::Ok || !serviceDoc.is<JsonObject>())
    {
        serviceDoc.clear();
        serviceDoc.to<JsonObject>();
    }
    JsonObject service = serviceDoc.as<JsonObject>();
    for (JsonPair entry : payload)
        service[entry.key()] = entry.value();
    const JsonArrayConst removedServers = payload["removed_servers"].as<JsonArrayConst>();
    for (JsonVariantConst removed : removedServers)
    {
        if (!removed.is<int>() || removed.as<int>() <= 2)
            continue;
        const auto suffix = String(removed.as<int>());
        const std::array<String, 9> keys = {{"server" + suffix + "_enabled", "server" + suffix + "_stream_on_demand",
                                             "server" + suffix + "_stream_active", "serverhost" + suffix,
                                             "port" + suffix, "mountpoint" + suffix, "username" + suffix,
                                             "password" + suffix, "ntrip_version" + suffix}};
        for (const String &key : keys)
            service.remove(key.c_str());
    }
    service.remove("removed_servers");
    const auto onDemand = jsonBool(service, "stream_on_demand", false);
    if (!onDemand)
        service["stream_active"] = true;
    else if (service["stream_active"].isNull())
        service["stream_active"] = false;
    const auto globalActive = jsonBool(service, "stream_active", false);
    for (uint8_t id = 1; id <= 2; ++id)
    {
        const auto prefix = String("server") + String(id);
        const auto onDemandKey = prefix + "_stream_on_demand";
        const auto activeKey = prefix + "_stream_active";
        const auto serverOnDemand = jsonBool(service, onDemandKey, onDemand);
        service[onDemandKey.c_str()] = serverOnDemand;
        if (!serverOnDemand)
            service[activeKey.c_str()] = true;
        else if (service[activeKey.c_str()].isNull())
            service[activeKey.c_str()] = globalActive;
    }
    serviceConfig_.clear();
    serializeJson(serviceDoc, serviceConfig_);
    detail = "service config deployed";
    setupNTRIP();
    connectNTRIP();
    publishConfigState("service_config_state");
    return true;
}

bool BackendAgent::executeRawCommand(JsonDocument &doc, JsonObject payload,
                                     String &error, String &detail)
{
    JsonArray commands = payload["commands_b64"].as<JsonArray>();
    const JsonObject originalConfig = doc["original_config"].as<JsonObject>();
    const auto configuringBase = String(originalConfig["mode"] | "") == "BASE";
    const auto sensorType = originalConfig["sensor_type"] | sensorType_;
    if (workflowActive())
    {
        requestWorkflowStop();
        const uint32_t stopWaitMs = millis();
        while (workflowActive() && static_cast<uint32_t>(millis() - stopWaitMs) < 1500UL)
            vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (workflowActive())
    {
        error = "cannot stop active base workflow";
        detail = error;
        return false;
    }
    if (configuringBase && sensorType != "Ublox" && sensorType != "Unicorecomm")
    {
        error = "unsupported GNSS sensor_type";
        detail = error;
        return false;
    }
    if (!executeRawCommands(commands, error, configuringBase ? sensorType : String()))
    {
        detail = error;
        return false;
    }
    detail = "raw commands executed";
    if (configuringBase)
    {
        sensorType_ = sensorType;
        baseStartupState_ = "configured";
        baseStartupError_ = "";
        baseStartupAttempted_ = true;
        autoBaseProgress_.clear();
        lastReceiverPollMs_ = 0;
        setAgentState(normalAgentState());
    }
    if (!originalConfig.isNull() && originalConfig.containsKey("params"))
    {
        DynamicJsonDocument baseDoc(CONFIG_DOCUMENT_SIZE);
        baseDoc.set(originalConfig["params"]);
        JsonObject base = baseDoc.as<JsonObject>();
        if (originalConfig.containsKey("mode"))
            base["mode"] = originalConfig["mode"];
        baseConfig_.clear();
        serializeJson(baseDoc, baseConfig_);
        publishConfigState("base_config_state");
    }
    return true;
}

bool BackendAgent::executeAutoBaseCommand(JsonObject payload, String &error,
                                          String &detail)
{
    const auto sensorType = payload["sensor_type"] | sensorType_;
    const auto requestedPort = payload["port"] | 0;
    const auto hasConnection = !String(payload["ip"] | String()).isEmpty() && !String(payload["user"] | String()).isEmpty() && !String(payload["password"] | String()).isEmpty() && !String(payload["mountpoint"] | String()).isEmpty() && requestedPort >= 1 && requestedPort <= 65535;
    if (workflowActive())
    {
        error = "another base workflow is already running";
        return false;
    }
    if (sensorType != "Ublox" && sensorType != "Unicorecomm")
    {
        error = "select Ublox or Unicorecomm in CORS before starting auto base";
        return false;
    }
    if (!hasConnection)
    {
        error = "missing or invalid CORS connection fields";
        return false;
    }
    sensorType_ = sensorType;
    String workflowPayload;
    serializeJson(payload, workflowPayload);
    if (!startWorkflow("auto_base", workflowPayload))
    {
        error = "cannot start auto base workflow";
        return false;
    }
    DynamicJsonDocument progress(CONFIG_DOCUMENT_SIZE);
    progress["ntrip_host"] = payload["ip"];
    progress["ntrip_port"] = payload["port"];
    progress["mountpoint"] = payload["mountpoint"];
    progress["note"] = "ESP32 forwards opaque RTCM and uses GNSS RTK/GGA; no RTCM decoder is running on the ESP32.";
    String progressJson;
    serializeJson(progress, progressJson);
    setAgentState("auto_setup_queued");
    setWorkflowProgress("queued", 0, 4, progressJson);
    detail = "auto base workflow queued";
    return true;
}

bool BackendAgent::executeReferenceCheckCommand(JsonObject payload, String &error,
                                                String &detail)
{
    const JsonObject reference = payload["reference_coords"].as<JsonObject>();
    const auto requestedPort = payload["port"] | 0;
    const auto validReference = !reference.isNull() && !reference["lat"].isNull() && !reference["lon"].isNull() && !reference["alt"].isNull();
    const auto hasConnection = !String(payload["ip"] | String()).isEmpty() && !String(payload["user"] | String()).isEmpty() && !String(payload["password"] | String()).isEmpty() && !String(payload["mountpoint"] | String()).isEmpty() && requestedPort >= 1 && requestedPort <= 65535;
    if (workflowActive())
    {
        error = "another base workflow is already running";
        return false;
    }
    if (!validReference || !hasConnection)
    {
        error = "missing or invalid reference/CORS configuration";
        return false;
    }
    String workflowPayload;
    serializeJson(payload, workflowPayload);
    if (!startWorkflow("reference_check", workflowPayload))
    {
        error = "cannot start reference check workflow";
        return false;
    }
    setAgentState("reference_check_queued");
    setWorkflowProgress("reference_check_queued", 0, 4);
    detail = "base reference check queued";
    return true;
}

bool BackendAgent::executeProvinceCommand(JsonObject payload, String &error,
                                          String &detail)
{
    auto province = payload["province_code"] | String();
    province.toUpperCase();
    double centralMeridianDeg = 0.0;
    double scale = 0.0;
    String provinceName;
    DynamicJsonDocument baseDoc(CONFIG_DOCUMENT_SIZE);
    const auto baseParsed = deserializeJson(baseDoc, baseConfig_) == DeserializationError::Ok && baseDoc.is<JsonObject>();
    JsonObject base = baseDoc.as<JsonObject>();
    JsonObject rawCoordinates = base["auto_base_raw_itrf_llh"].as<JsonObject>();
    if (rawCoordinates.isNull())
        rawCoordinates = base["coords"].as<JsonObject>();
    if (rawCoordinates.isNull())
        rawCoordinates = base;
    const auto rawLatitude = rawCoordinates["lat"] | NAN;
    const auto rawLongitude = rawCoordinates["lon"] | NAN;
    const auto rawAltitude = rawCoordinates["alt"] | NAN;
    double localLatitude = rawLatitude;
    double localLongitude = rawLongitude;
    double localAltitude = rawAltitude;
    double northing = 0.0;
    double easting = 0.0;
    if (province.isEmpty())
    {
        error = "missing payload.province_code";
        return false;
    }
    if (!getProvinceProjection(province, centralMeridianDeg, scale, provinceName))
    {
        error = "invalid province_code";
        return false;
    }
    const auto validCoordinates = baseParsed && std::isfinite(rawLatitude) && std::isfinite(rawLongitude) && std::isfinite(rawAltitude) && transformToVn2000(rawLatitude, rawLongitude, rawAltitude, centralMeridianDeg, scale, localLatitude, localLongitude, localAltitude, northing, easting);
    if (!validCoordinates)
    {
        error = "base coordinates are missing or cannot be transformed";
        return false;
    }
    if (!applyBaseCoordinates(localLatitude, localLongitude, localAltitude))
    {
        error = "cannot apply transformed coordinates to GNSS";
        return false;
    }
    JsonObject coordinates = base["coords"].as<JsonObject>();
    if (coordinates.isNull())
        coordinates = base.createNestedObject("coords");
    coordinates["lat"] = localLatitude;
    coordinates["lon"] = localLongitude;
    coordinates["alt"] = localAltitude;
    base["itrf_vn2000_transform_applied"] = true;
    base["central_meridian_deg"] = centralMeridianDeg;
    base["k0"] = scale;
    base["province_code"] = province;
    base["province_name"] = provinceName;
    base["projection_source"] = "manual_province_override";
    base["vn2000_northing"] = northing;
    base["vn2000_easting"] = easting;
    baseConfig_.clear();
    serializeJson(baseDoc, baseConfig_);
    saveStatePreferences();
    publishConfigState("base_config_state");
    setWorkflowProgress("reprojected", 4, 4);
    detail = "VN2000 province transform applied";
    return true;
}

bool BackendAgent::executeBasicCommand(const String &command, JsonObject payload,
                                       String &error, String &detail)
{
    if (command == "LOCK_DEVICE")
    {
        if (!applyLockCommand(payload, error))
            return false;
        setAgentState(normalAgentState());
        detail = "device lock updated";
        return true;
    }
    if (command == "UNLOCK_DEVICE")
    {
        stationLock_.restore(false, 0);
        if (baseStartupState_ == "locked")
            baseStartupAttempted_ = false;
        setAgentState(normalAgentState());
        detail = "device unlocked";
        return true;
    }
    if (command != "PROVISION_DEVICE")
        return false;
    const auto name = payload["name"] | String();
    if (name.isEmpty())
    {
        error = "missing payload.name";
        return false;
    }
    deviceName_ = name;
    provisioned_ = true;
    setupNTRIP();
    connectNTRIP();
    detail = "device provisioned";
    return true;
}

bool BackendAgent::executeStreamCommand(JsonObject payload, String &error,
                                        String &detail)
{
    if (!payload.containsKey("active"))
    {
        error = "missing payload.active";
        return false;
    }
    applyServiceStreamCommand(payload);
    setupNTRIP();
    connectNTRIP();
    detail = (payload["active"] | false) ? "stream activated" : "stream set to standby";
    return true;
}

bool BackendAgent::executeBaseStatusCommand(String &error, String &detail)
{
    if (sensorType_ != "Unicorecomm" && sensorType_ != "Ublox")
    {
        error = "GNSS serial is busy";
        return false;
    }
    if (!refreshReceiverReadback(error))
        return false;
    detail = "receiver mode and position read back";
    return true;
}

bool BackendAgent::executeStopWorkflowCommand(String &detail)
{
    if (workflowActive())
    {
        requestWorkflowStop();
        setWorkflowProgress("stopping", 0, 4);
        detail = "base workflow stop requested";
        return true;
    }
    setWorkflowProgress("stopped", 0, 4);
    setAgentState(normalAgentState());
    detail = "auto base stopped";
    return true;
}

void BackendAgent::finishCommand(const String &source, const String &command,
                                 const String &commandId, bool success,
                                 const String &error, const String &detail,
                                 bool restartAfterCommand)
{
    const auto workflowState = command == "TRIGGER_AUTO_BASE" || command == "TRIGGER_BASE_REFERENCE_CHECK" || (command == "STOP_AUTO_BASE" && workflowActive());
    if (success && command != "REBOOT_DEVICE" && command != "DELETE_DEVICE" && command != "DEPLOY_LICENSE" && !workflowState)
    {
        setAgentState(normalAgentState());
    }
    if (!success)
        setAgentState(normalAgentState());
    recordCommandResult(command, source, success ? "success" : "error",
                        success ? detail : error, commandId);
    saveStatePreferences();
    publishStatus(true);
    if (success && restartAfterCommand)
    {
        delay(500);
        mqtt.disconnect();
        ESP.restart();
    }
}

void BackendAgent::executeCommand(const String &source, const String &command,
                                  const String &commandId, JsonDocument &doc,
                                  JsonObject payload)
{
    setAgentState("configuring");
    String error;
    auto success = true;
    String detail;
    auto restartAfterCommand = false;

    if (command == "LOCK_DEVICE" || command == "UNLOCK_DEVICE" || command == "PROVISION_DEVICE")
    {
        success = executeBasicCommand(command, payload, error, detail);
    }
    else if (command == "DEPLOY_LICENSE")
    {
        success = executeLicenseCommand(payload, error, detail, restartAfterCommand);
    }
    else if (command == "DEPLOY_SERVICE_CONFIG")
    {
        success = executeServiceConfigCommand(payload, error, detail);
    }
    else if (command == "SET_RTCM_STREAM_ACTIVE")
    {
        success = executeStreamCommand(payload, error, detail);
    }
    else if (command == "EXECUTE_RAW_COMMANDS")
    {
        success = executeRawCommand(doc, payload, error, detail);
    }
    else if (command == "CHECK_BASE_STATUS")
    {
        success = executeBaseStatusCommand(error, detail);
    }
    else if (command == "TRIGGER_AUTO_BASE")
    {
        success = executeAutoBaseCommand(payload, error, detail);
    }
    else if (command == "TRIGGER_BASE_REFERENCE_CHECK")
    {
        success = executeReferenceCheckCommand(payload, error, detail);
    }
    else if (command == "STOP_AUTO_BASE")
    {
        success = executeStopWorkflowCommand(detail);
    }
    else if (command == "APPLY_VN2000_PROVINCE")
    {
        success = executeProvinceCommand(payload, error, detail);
    }
    else if (command == "REBOOT_DEVICE")
    {
        requestWorkflowStop();
        setAgentState("rebooting");
        recordCommandResult(command, source, "success", "device reboot requested", commandId);
        saveStatePreferences();
        publishStatus(true);
        delay(500);
        mqtt.disconnect();
        ESP.restart();
    }
    else if (command == "DELETE_DEVICE")
    {
        requestWorkflowStop();
        setAgentState("rebooting_for_reset");
        recordCommandResult(command, source, "success", "device reset requested", commandId);
        saveStatePreferences();
        publishStatus(true);
        delay(500);
        prefs.begin("myPrefs", false);
        prefs.clear();
        prefs.end();
        if (beginBackendPreferences(false))
        {
            backendPreferences().clear();
            backendPreferences().end();
        }
        mqtt.disconnect();
        ESP.restart();
    }

    finishCommand(source, command, commandId, success, error, detail, restartAfterCommand);
}
