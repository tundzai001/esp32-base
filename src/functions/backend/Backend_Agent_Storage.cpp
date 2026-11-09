#include "functions/backend/Backend_Agent.h"
#include "functions/backend/Backend_Agent_Internal.h"
#include "Prog_Config.h"
#include <nvs.h>
#include <esp_err.h>

using namespace BackendAgentInternal;
extern Preferences prefs;

namespace BackendAgentInternal {
namespace {
constexpr char BACKEND_PREFS_NAMESPACE[] = "backend";
constexpr char BACKEND_PREFS_PARTITION[] = "agent_state";
Preferences backendPrefs;
bool useDedicatedBackendPrefs = false;

String restoreWorkflowProgress(const String &saved) {
    DynamicJsonDocument progress(CONFIG_DOCUMENT_SIZE);
    if (deserializeJson(progress, saved) != DeserializationError::Ok) return "{}";

    // Only completed reports survive boot. A running workflow owns a RAM-only task.
    const String phase = progress["phase"] | String();
    if (phase == "ready_for_review" || phase == "completed" || phase == "failed"
        || phase == "stopped" || phase == "reference_check_completed"
        || phase == "reference_check_failed") {
        return saved;
    }
    return "{}";
}
} // namespace

const char *backendPreferencesPartition() {
    return useDedicatedBackendPrefs ? BACKEND_PREFS_PARTITION : "nvs";
}

Preferences &backendPreferences() {
    return useDedicatedBackendPrefs ? backendPrefs : prefs;
}

bool beginBackendPreferences(bool readOnly) {
    return useDedicatedBackendPrefs
        ? backendPrefs.begin(BACKEND_PREFS_NAMESPACE, readOnly, BACKEND_PREFS_PARTITION)
        : prefs.begin("myPrefs", readOnly);
}

static bool migrateBackendPreferences() {
    if (!backendPrefs.begin(BACKEND_PREFS_NAMESPACE, false, BACKEND_PREFS_PARTITION)) {
        return false;
    }
    const bool alreadyMigrated = backendPrefs.isKey("MIGRATED");
    backendPrefs.end();
    if (alreadyMigrated) return true;

    if (!prefs.begin("myPrefs", true)) return false;
    if (!backendPrefs.begin(BACKEND_PREFS_NAMESPACE, false, BACKEND_PREFS_PARTITION)) {
        prefs.end();
        return false;
    }

    auto copyString = [](const char *key) {
        if (!prefs.isKey(key)) return true;
        const String value = prefs.getString(key);
        return backendPrefs.putString(key, value) == value.length();
    };
    auto copyBool = [](const char *key) {
        if (!prefs.isKey(key)) return true;
        return backendPrefs.putBool(key, prefs.getBool(key)) == 1;
    };

    bool migrated = true;
    migrated &= copyString("DEVICE_SERIAL");
    migrated &= copyString("DEVICE_NAME");
    migrated &= copyString("SENSOR_TYPE");
    migrated &= copyBool("PROVISIONED");
    migrated &= copyBool("REMOTE_LOCK");
    if (prefs.isKey("LOCK_SERVERS")) {
        migrated &= backendPrefs.putUChar("LOCK_SERVERS", prefs.getUChar("LOCK_SERVERS")) == 1;
    }
    migrated &= copyString("BASE_JSON");
    migrated &= copyString("SERVICE_JSON");
    migrated &= copyString("LICENSE_KEY");
    migrated &= copyString("CMD_RESULT");
    migrated &= copyString("PROGRESS_JSON");
    if (migrated) migrated = backendPrefs.putBool("MIGRATED", true) == 1;
    prefs.end();
    backendPrefs.end();
    Serial.printf("[BACKEND][NVS] Migrasi state sang agent_state: %s.\n",
                  migrated ? "THANH CONG" : "THAT BAI");
    return migrated;
}

String readPreference(const char *key, const String &fallback) {
    prefs.begin("myPrefs", true);
    const String value = prefs.isKey(key) ? prefs.getString(key) : fallback;
    prefs.end();
    return value;
}

} // namespace BackendAgentInternal

void BackendAgent::loadPreferences() {
    useDedicatedBackendPrefs = migrateBackendPreferences();
    if (!beginBackendPreferences(true)) {
        useDedicatedBackendPrefs = false;
        prefs.begin("myPrefs", true);
        Serial.println("[BACKEND][NVS] Khong mo duoc agent_state; dung NVS cu.");
    }
    Preferences &statePrefs = backendPreferences();

    serialNumber_ = statePrefs.getString("DEVICE_SERIAL", String());
    bool serialNeedsSave = false;
    if (serialNumber_.isEmpty()) {
        uint64_t mac = 0;
#if !defined(NATIVE_BUILD)
        mac = ESP.getEfuseMac();
#endif
        char generated[24] = {};
        snprintf(generated, sizeof(generated), "esp_%012llX",
                 static_cast<unsigned long long>(mac & 0xFFFFFFFFFFFFULL));
        serialNumber_ = String(generated);
        serialNeedsSave = true;
    } else if (serialNumber_.startsWith("LP_")) {
        // Replace the legacy LP_ prefix while preserving the device identifier suffix.
        serialNumber_ = String("esp_") + serialNumber_.substring(3);
        serialNeedsSave = true;
    }

    deviceName_ = statePrefs.getString("DEVICE_NAME", "ESP32-GNSS");
    sensorType_ = statePrefs.getString("SENSOR_TYPE", "UNKNOWN");
    provisioned_ = statePrefs.getBool("PROVISIONED", false);
    stationLock_.restore(statePrefs.getBool("REMOTE_LOCK", false),
                         statePrefs.getUChar("LOCK_SERVERS", 0));
    baseConfig_ = statePrefs.getString("BASE_JSON", "{}");
    serviceConfig_ = statePrefs.getString("SERVICE_JSON", "{}");
    licenseToken_ = statePrefs.getString("LICENSE_KEY", "");
    lastCommandResult_ = statePrefs.getString("CMD_RESULT", "{}");
    autoBaseProgress_ = restoreWorkflowProgress(statePrefs.getString("PROGRESS_JSON", "{}"));
    surveyInResult_ = statePrefs.isKey("SURVEY_RESULT") ? statePrefs.getString("SURVEY_RESULT") : "{}";
    statePrefs.end();
    prefs.begin("myPrefs", true);
    const auto bootCounter = static_cast<uint32_t>(prefs.getInt("RSTRT_COUNT", 0));
    prefs.end();
    if (serialNeedsSave) {
        const bool opened = beginBackendPreferences(false);
        const bool saved = opened
            && backendPreferences().putString("DEVICE_SERIAL", serialNumber_) == serialNumber_.length();
        if (opened) backendPreferences().end();
        Serial.printf("[BACKEND] Serial %s: %s\n", serialNumber_.c_str(), saved ? "da luu NVS" : "LOI luu NVS");
    }
    licenseValid_ = licenseMatchesSerial(serialNumber_, licenseToken_);
    Serial.printf("[BACKEND] Device Serial: %s\n", serialNumber_.c_str());
    Serial.printf("[BACKEND][LICENSE] NVS: %s; kiem tra: %s\n",
                  licenseToken_.isEmpty() ? "chua co key" : "da co key",
                  licenseValid_ ? "HOP LE" : "KHONG HOP LE");

    char bootId[40] = {};
    snprintf(bootId, sizeof(bootId), "%s-%08lx", serialNumber_.c_str(),
             static_cast<unsigned long>(bootCounter));
    statusBootId_ = String(bootId);
    agentState_ = normalAgentState();
}

void BackendAgent::ensureDefaultServiceConfig() {
    DynamicJsonDocument parsed(CONFIG_DOCUMENT_SIZE);
    if (deserializeJson(parsed, serviceConfig_) == DeserializationError::Ok
        && parsed.is<JsonObject>()
        && parsed.as<JsonObjectConst>().size() > 0) {
        return;
    }

    prefs.begin("myPrefs", true);
    const String host = prefs.getString("NTRIP_SERVER", String(NTRIP_CASTER_ADDRESS));
    const uint16_t port = prefs.getUShort("NTRIP_PORT", NTRIP_CASTER_PORT);
    String mountpoint = prefs.getString("NTRIP_MPT", String(NTRIP_MOUNTPOINT));
    const String password = prefs.getString("NT_AUTH_BS", String(NTRIP_AUTH_BASE_STATION));
    prefs.end();
    while (mountpoint.startsWith("/")) mountpoint.remove(0, 1);

    DynamicJsonDocument defaults(CONFIG_DOCUMENT_SIZE);
    JsonObject service = defaults.to<JsonObject>();
    service["stream_on_demand"] = false;
    service["stream_active"] = true;
    service["server1_enabled"] = !host.isEmpty();
    service["server1_stream_on_demand"] = false;
    service["server1_stream_active"] = true;
    service["ntrip_version1"] = 1;
    service["serverhost1"] = host;
    service["port1"] = port;
    service["username1"] = "source";
    service["password1"] = password;
    service["mountpoint1"] = mountpoint;
    service["server2_enabled"] = false;
    service["server2_stream_on_demand"] = false;
    service["server2_stream_active"] = false;
    serviceConfig_.clear();
    serializeJson(defaults, serviceConfig_);
    Serial.printf("[BACKEND][NVS] Default SERVICE_JSON (%u bytes): %s.\n",
                  static_cast<unsigned>(serviceConfig_.length()),
                  saveStatePreferences() ? "da luu" : "LOI luu");
}

bool BackendAgent::saveStatePreferences() {
    if (serialNumber_.isEmpty()) {
        Serial.println("[BACKEND][NVS] Refusing to save state before agent identity is loaded.");
        return false;
    }
    // Persist the state snapshot with native NVS calls and one commit.
    nvs_handle_t handle;
    const esp_err_t opened = useDedicatedBackendPrefs
        ? nvs_open_from_partition(BACKEND_PREFS_PARTITION, BACKEND_PREFS_NAMESPACE, NVS_READWRITE, &handle)
        : nvs_open("myPrefs", NVS_READWRITE, &handle);
    if (opened != ESP_OK) {
        Serial.printf("[BACKEND][NVS] Open failed: %s (0x%x)\n", esp_err_to_name(opened), opened);
        return false;
    }
    bool saved = true;
    const auto check = [&](const char *key, esp_err_t error) {
        if (error != ESP_OK) {
            Serial.printf("[BACKEND][NVS] Write %s failed: %s (0x%x)\n", key, esp_err_to_name(error), error);
            saved = false;
        }
    };
    check("DEVICE_SERIAL", nvs_set_str(handle, "DEVICE_SERIAL", serialNumber_.c_str()));
    check("DEVICE_NAME", nvs_set_str(handle, "DEVICE_NAME", deviceName_.c_str()));
    check("SENSOR_TYPE", nvs_set_str(handle, "SENSOR_TYPE", sensorType_.c_str()));
    check("PROVISIONED", nvs_set_u8(handle, "PROVISIONED", provisioned_));
    check("REMOTE_LOCK", nvs_set_u8(handle, "REMOTE_LOCK", stationLock_.all));
    check("LOCK_SERVERS", nvs_set_u8(handle, "LOCK_SERVERS", stationLock_.servers));
    check("BASE_JSON", nvs_set_str(handle, "BASE_JSON", baseConfig_.c_str()));
    check("SERVICE_JSON", nvs_set_str(handle, "SERVICE_JSON", serviceConfig_.c_str()));
    check("LICENSE_KEY", nvs_set_str(handle, "LICENSE_KEY", licenseToken_.c_str()));
    check("CMD_RESULT", nvs_set_str(handle, "CMD_RESULT", lastCommandResult_.c_str()));
    check("PROGRESS_JSON", nvs_set_str(handle, "PROGRESS_JSON", autoBaseProgress_.c_str()));
    check("SURVEY_RESULT", nvs_set_str(handle, "SURVEY_RESULT", surveyInResult_.c_str()));
    check("commit", nvs_commit(handle));
    nvs_close(handle);
    return saved;
}
