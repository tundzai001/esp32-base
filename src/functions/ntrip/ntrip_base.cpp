#include "functions/ntrip/ntrip_base.h"
#include "Prog_Config.h"
#include "functions/backend/Backend_Agent.h"
#include "functions/backend/Backend_Agent_Internal.h"
#include <ArduinoJson.h>
#include <Preferences.h>
#include <array>

extern Preferences prefs;

using BackendAgentInternal::jsonBool;
using BackendAgentInternal::jsonString;
using BackendAgentInternal::encodeBase64;
using namespace ntrip_common;

namespace {
constexpr size_t CONFIG_DOCUMENT_SIZE = 4096;
constexpr uint8_t NTRIP_SERVER_COUNT = 2;
constexpr uint32_t RECONNECT_INTERVAL_MS = 5000;
constexpr uint32_t BPS_WINDOW_MS = 1000;

struct NtripEndpoint {
    bool enabled = false;
    bool authenticated = false;
    String host;
    uint16_t port = 2101;
    String mountpoint;
    String username;
    String password;
    uint8_t version = 1;
    uint32_t lastReconnectMs = 0;
    uint32_t bytesInWindow = 0;
    uint32_t bytesPerSecond = 0;
    uint32_t windowStartedMs = 0;
    uint32_t lastKeepAliveMs = 0;
    String lastError;
};

std::array<NtripEndpoint, NTRIP_SERVER_COUNT> endpoints;
void clearEndpointState(NtripEndpoint &endpoint) {
    endpoint.authenticated = false;
    endpoint.bytesInWindow = 0;
    endpoint.bytesPerSecond = 0;
    endpoint.windowStartedMs = millis();
    endpoint.lastKeepAliveMs = endpoint.windowStartedMs;
}

struct LocalNtripConfig
{
    String host;
    String mountpoint;
    String password;
    uint16_t port = NTRIP_CASTER_PORT;
};

void loadLocalNtripConfig(LocalNtripConfig &legacy)
{
    prefs.begin("myPrefs", true);
    legacy.host = prefs.getString("NTRIP_SERVER", String(NTRIP_CASTER_ADDRESS));
    legacy.port = prefs.getUShort("NTRIP_PORT", NTRIP_CASTER_PORT);
    legacy.mountpoint = prefs.getString("NTRIP_MPT", String(NTRIP_MOUNTPOINT));
    legacy.password = prefs.getString("NT_AUTH_BS", String(NTRIP_AUTH_BASE_STATION));
    prefs.end();
}

String endpointHost(JsonObjectConst service, uint8_t index)
{
    const auto suffix = String(index + 1);
    const auto generic = index == 0 ? "ip" : "ip2";
    const auto fallback = index == 0 ? jsonString(service, "host", String()) : String();
    return jsonString(service, "serverhost" + suffix,
                      jsonString(service, "server" + suffix + "_host",
                                 jsonString(service, "host" + suffix,
                                            jsonString(service, generic, fallback))));
}

uint16_t endpointPort(JsonObjectConst service, uint8_t index, uint16_t legacyPort)
{
    const auto suffix = String(index + 1);
    const auto generic = index == 0 ? "port" : "port2";
    const uint16_t fallback = index == 0 ? legacyPort : 2101;
    return static_cast<uint16_t>(service["port" + suffix] | (service["serverport" + suffix] | (service[generic] | fallback)));
}

void normalizeEndpoint(NtripEndpoint &endpoint)
{
    while (endpoint.mountpoint.startsWith("/"))
        endpoint.mountpoint.remove(0, 1);
    endpoint.enabled = endpoint.enabled && !endpoint.host.isEmpty() && !endpoint.mountpoint.isEmpty() && endpoint.port > 0 && safeNtripField(endpoint.host) && safeNtripField(endpoint.mountpoint) && safeNtripField(endpoint.username) && safeNtripField(endpoint.password);
}

void loadEndpoint(NtripEndpoint &endpoint, uint8_t index, JsonObjectConst service,
                  bool hasServiceConfig, const LocalNtripConfig &legacy)
{
    const auto suffix = String(index + 1);
    auto genericUser = "user2";
    auto genericPassword = "password2";
    auto genericMountpoint = "mountpoint2";
    String legacyHost;
    String legacyMountpoint;
    String legacyPassword;
    uint16_t legacyPort = 2101;
    if (index == 0)
    {
        genericUser = "user";
        genericPassword = "password";
        genericMountpoint = "mountpoint";
        legacyHost = legacy.host;
        legacyMountpoint = legacy.mountpoint;
        legacyPassword = legacy.password;
        legacyPort = legacy.port;
    }

    if (hasServiceConfig)
    {
        endpoint.enabled = jsonBool(service, "server" + suffix + "_enabled", false);
        endpoint.host = endpointHost(service, index);
        endpoint.port = endpointPort(service, index, legacy.port);
        endpoint.mountpoint = jsonString(service, "mountpoint" + suffix,
                                         jsonString(service, "mp" + suffix,
                                                    jsonString(service, genericMountpoint, String())));
        endpoint.username = jsonString(service, "username" + suffix,
                                       jsonString(service, "user" + suffix,
                                                  jsonString(service, genericUser, "source")));
        endpoint.password = jsonString(service, "password" + suffix,
                                       jsonString(service, "pass" + suffix,
                                                  jsonString(service, genericPassword, legacyPassword)));
        endpoint.version = static_cast<uint8_t>(service["ntrip_version" + suffix] | 1);
    }
    else
    {
        endpoint.enabled = index == 0 && !legacy.host.isEmpty();
        endpoint.host = legacyHost;
        endpoint.port = legacyPort;
        endpoint.mountpoint = legacyMountpoint;
        endpoint.username = "source";
        endpoint.password = legacyPassword;
        endpoint.version = 1;
    }
    normalizeEndpoint(endpoint);
}

void loadEndpoints()
{
    DynamicJsonDocument serviceDoc(CONFIG_DOCUMENT_SIZE);
    const auto serialized = backendAgent.serviceConfigJson();
    const auto hasServiceConfig = deserializeJson(serviceDoc, serialized) == DeserializationError::Ok && serviceDoc.is<JsonObject>();
    const JsonObjectConst service = serviceDoc.as<JsonObjectConst>();

    LocalNtripConfig legacy;
    loadLocalNtripConfig(legacy);

    for (uint8_t index = 0; index < NTRIP_SERVER_COUNT; ++index)
    {
        loadEndpoint(endpoints[index], index, service, hasServiceConfig, legacy);
    }
}

bool streamActive(uint8_t index) {
    if (index >= NTRIP_SERVER_COUNT || backendDeviceLocked(index + 1)
        || !backendLicenseValid() || !backendDeviceProvisioned() || !backendBaseStartupConfigured()) return false;
    DynamicJsonDocument serviceDoc(CONFIG_DOCUMENT_SIZE);
    if (deserializeJson(serviceDoc, backendAgent.serviceConfigJson()) != DeserializationError::Ok
        || !serviceDoc.is<JsonObject>()) {
        return index == 0;
    }
    return BackendAgent::effectiveStreamActive(
        serviceDoc.as<JsonObjectConst>(), index + 1, backendControlPlaneFailOpen());
}

void stopEndpoint(uint8_t index) {
    clientFor(index).stop();
    clearEndpointState(endpoints[index]);
}

bool connectEndpoint(uint8_t index) {
    if (index >= NTRIP_SERVER_COUNT || !endpoints[index].enabled || !streamActive(index)) return false;
    NtripEndpoint &endpoint = endpoints[index];
    Client &client = clientFor(index);
    endpoint.lastReconnectMs = millis();
    client.stop();
    client.setTimeout(3000);

    Serial.println("[NTRIP] Ket noi S" + String(index + 1) + ": "
                   + endpoint.host + ":" + String(endpoint.port)
                   + "/" + endpoint.mountpoint);
    if (!client.connect(endpoint.host.c_str(), endpoint.port)) {
        endpoint.lastError = "TCP connection failed";
        Serial.printf("[NTRIP][ERROR] S%u: %s\n", index + 1, endpoint.lastError.c_str());
        clearEndpointState(endpoint);
        return false;
    }

    const String user = endpoint.username.isEmpty() ? String("source") : endpoint.username;
    const String auth = encodeBase64(user + ":" + endpoint.password);
    String request;
    if (endpoint.version == 2) {
        request = "POST /" + endpoint.mountpoint + " HTTP/1.1\r\n"
                  "Host: " + endpoint.host + ":" + String(endpoint.port) + "\r\n"
                  "Ntrip-Version: Ntrip/2.0\r\n"
                  "User-Agent: NTRIP ESP32-Agent/" + String(AGENT_VERSION) + "\r\n"
                  "Authorization: Basic " + auth + "\r\n"
                  "Transfer-Encoding: chunked\r\n"
                  "Connection: Keep-Alive\r\n\r\n";
    } else {
        request = "SOURCE " + endpoint.password + " /" + endpoint.mountpoint + " HTTP/1.1\r\n"
                  "Host: " + endpoint.host + ":" + String(endpoint.port) + "\r\n"
                  "Ntrip-Version: Ntrip/2.0\r\n"
                  "User-Agent: NTRIP ESP32-Agent/" + String(AGENT_VERSION) + "\r\n"
                  "Authorization: Basic " + auth + "\r\n"
                  "Connection: Keep-Alive\r\n\r\n";
    }
    client.print(request);

    String responseLine;
    if (!waitForNtripResponse(client, &responseLine)) {
        if (!endpoint.password.isEmpty()) responseLine.replace(endpoint.password, "[redacted]");
        endpoint.lastError = responseLine;
        Serial.printf("[NTRIP][ERROR] Caster tu choi S%u: %s\n", index + 1, responseLine.c_str());
        client.stop();
        clearEndpointState(endpoint);
        return false;
    }

    endpoint.authenticated = true;
    endpoint.lastError = "";
    endpoint.windowStartedMs = millis();
    endpoint.bytesInWindow = 0;
    endpoint.bytesPerSecond = 0;
    endpoint.lastKeepAliveMs = endpoint.windowStartedMs;
    Serial.println("[NTRIP] Xac thuc thanh cong S" + String(index + 1));
    return true;
}

void updateBps(NtripEndpoint &endpoint) {
    const uint32_t now = millis();
    if (endpoint.windowStartedMs == 0) endpoint.windowStartedMs = now;
    if (now - endpoint.windowStartedMs >= BPS_WINDOW_MS) {
        const uint32_t elapsed = now - endpoint.windowStartedMs;
        endpoint.bytesPerSecond = elapsed > 0
            ? static_cast<uint32_t>((static_cast<uint64_t>(endpoint.bytesInWindow) * 1000ULL) / elapsed)
            : endpoint.bytesInWindow;
        endpoint.bytesInWindow = 0;
        endpoint.windowStartedMs = now;
    }
}

} // namespace

Client &activeNtripClient() {
    return clientFor(0);
}

int setupNTRIP() {
    for (uint8_t index = 0; index < NTRIP_SERVER_COUNT; ++index) stopEndpoint(index);
    loadEndpoints();
    return 0;
}

int connectNTRIP() {
    loadEndpoints();
    int connected = 0;
    for (uint8_t index = 0; index < NTRIP_SERVER_COUNT; ++index) {
        if (connectEndpoint(index)) ++connected;
    }
    return connected > 0 ? 0 : -1;
}

static bool sendEndpointRtcm(NtripEndpoint &endpoint, Client &client, const String &rtcmData) {
    if (rtcmData.isEmpty()) {
        if (endpoint.version == 2 && millis() - endpoint.lastKeepAliveMs >= 10000UL) {
            client.print("0\r\n\r\n");
            endpoint.lastKeepAliveMs = millis();
        }
        return false;
    }
    const size_t payloadLength = rtcmData.length();
    if (endpoint.version == 2) client.print(String(payloadLength, HEX) + "\r\n");
    const size_t sent = client.write(reinterpret_cast<const uint8_t *>(rtcmData.c_str()), payloadLength);
    if (endpoint.version == 2) client.print("\r\n");
    if (sent != payloadLength) return false;
    endpoint.bytesInWindow += static_cast<uint32_t>(sent);
    return true;
}

int loopNTRIP(String &rtcmData) {
    if (roverSessionActive || backendAgent.workflowActive()) return 504;

    int returnCode = NTRIP_MODE;
    bool anyConnected = false;
    bool anySent = false;

    for (uint8_t index = 0; index < NTRIP_SERVER_COUNT; ++index) {
        NtripEndpoint &endpoint = endpoints[index];
        Client &client = clientFor(index);
        if (!endpoint.enabled || !streamActive(index)) {
            if (client.connected()) client.stop();
            clearEndpointState(endpoint);
            continue;
        }

        if (!client.connected() || !endpoint.authenticated) {
            endpoint.authenticated = false;
            if (millis() - endpoint.lastReconnectMs >= RECONNECT_INTERVAL_MS
                    && connectEndpoint(index))
                anyConnected = true;
            continue;
        }

        anyConnected = true;
        anySent = sendEndpointRtcm(endpoint, client, rtcmData) || anySent;
        updateBps(endpoint);
    }

    if (anySent) returnCode += 4;
    if (!anyConnected) returnCode = 504;
    return returnCode;
}

void stopNtrip() {
    for (uint8_t index = 0; index < NTRIP_SERVER_COUNT; ++index) stopEndpoint(index);
    roverSessionActive = false;
    roverSessionAuthenticated = false;
    roverLastGgaMs = 0;
}

bool isNtripConnected() {
    return ntripServerConnected(1) || ntripServerConnected(2);
}

bool ntripServerConnected(uint8_t serverId) {
    if (serverId < 1 || serverId > NTRIP_SERVER_COUNT) return false;
    const uint8_t index = serverId - 1;
    return endpoints[index].authenticated && clientFor(index).connected();
}

uint32_t ntripServerBps(uint8_t serverId) {
    if (serverId < 1 || serverId > NTRIP_SERVER_COUNT) return 0;
    updateBps(endpoints[serverId - 1]);
    return endpoints[serverId - 1].bytesPerSecond;
}

String ntripServerError(uint8_t serverId) {
    if (serverId < 1 || serverId > NTRIP_SERVER_COUNT) return String();
    return endpoints[serverId - 1].lastError;
}
