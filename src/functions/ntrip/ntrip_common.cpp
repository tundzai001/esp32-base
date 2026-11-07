#define NTRIP_HANDLER_IP_CODE

#include "functions/ntrip/ntrip_common.h"
#include "functions/ntrip/unicore_bootstrap_cmds.h"

#include "Prog_Config.h"
#include "functions/backend/Backend_Agent.h"
#include "functions/ntrip/RTCM_Receiver.h"
#include "functions/gnss/unicore_commands.h"
#include "functions/gnss/Unicore_Identification.h"
#include "hardware/Sim_handler.h"
#include "hardware/Wifi_handler.h"

#include <ArduinoJson.h>
#include <Preferences.h>
#include <array>
#include <cstring>

extern Preferences prefs;
extern TinyGsm modem;

namespace
{
    class ntripConfig
    {
    public:
        inline static bool isIcyOk = false;
        inline static unsigned long lastReconnect = 0;
        inline static bool isNmeaSent = false;
    };

    inline constexpr std::array<uint8_t, 8> ubxMonVerPoll = {
        0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34};

    void waitForUm980Input()
    {
        const auto started = millis();
        while (!Serial1.available() && millis() - started < 2000)
            delay(10);
    }

    void logUm980Response()
    {
#ifdef PROGRAM_DEBUG
        waitForUm980Input();
        const auto response = Serial1.readStringUntil('\n');
        Serial.println("[NMEA CMD] Response: " + response);
#endif
    }

    inline void sendUm980Command(const char *command, size_t maxLength, bool logResponse,
                                 uint32_t pauseMs)
    {
        waitForUm980Input();
        Serial1.write(command, strnlen(command, maxLength));
        if (logResponse)
            logUm980Response();
        if (pauseMs > 0)
            delay(pauseMs);
    }

    void clearGnssInput()
    {
        while (Serial1.available() > 0)
        {
            Serial1.read();
        }
    }

    bool hasValidUbxFrame(const uint8_t *data, size_t length)
    {
        for (size_t start = 0; start + 8 <= length; ++start)
        {
            if (data[start] != 0xB5 || data[start + 1] != 0x62 ||
                data[start + 2] != 0x0A || data[start + 3] != 0x04)
            {
                continue;
            }

            const auto payloadLength =
                static_cast<size_t>(data[start + 4]) |
                (static_cast<size_t>(data[start + 5]) << 8);
            const auto frameLength = payloadLength + 8;
            // A MON-VER response contains SW (30 bytes) and HW (10 bytes), never an empty poll.
            if (payloadLength < 40 || frameLength > 512)
                continue;
            if (start + frameLength > length)
            {
                continue;
            }

            uint8_t checksumA = 0;
            uint8_t checksumB = 0;
            for (size_t i = start + 2; i < start + 6 + payloadLength; ++i)
            {
                checksumA = static_cast<uint8_t>(checksumA + data[i]);
                checksumB = static_cast<uint8_t>(checksumB + checksumA);
            }

            if (checksumA == data[start + frameLength - 2] &&
                checksumB == data[start + frameLength - 1])
            {
                return true;
            }
        }
        return false;
    }

    bool receiveUbxFrame(uint32_t timeoutMs)
    {
        std::array<uint8_t, 512> buffer{};
        auto length = 0;
        const uint32_t start = millis();

        while (millis() - start < timeoutMs)
        {
            while (Serial1.available() > 0)
            {
                const auto value = Serial1.read();
                if (length < buffer.size())
                {
                    buffer[length++] = static_cast<uint8_t>(value);
                }
                else
                {
                    memmove(buffer.data(), buffer.data() + 1, buffer.size() - 1);
                    buffer[buffer.size() - 1] = static_cast<uint8_t>(value);
                }
                if (hasValidUbxFrame(buffer.data(), length))
                {
                    return true;
                }
            }
            delay(2);
        }
        return false;
    }

    bool receiveUnicoreIdentification(uint32_t timeoutMs)
    {
        UnicoreIdentification identity;
        const uint32_t start = millis();
        while (millis() - start < timeoutMs)
        {
            while (Serial1.available() > 0)
            {
                if (identity.feed(static_cast<uint8_t>(Serial1.read())))
                {
                    Serial.printf("[GNSS][UNICORE RESPONSE] %s\n", identity.line.data());
                    return true;
                }
            }
            delay(2);
        }

        return false;
    }
} // namespace

bool jsonBool(JsonObjectConst object, const String &key, bool fallback = false)
{
    const JsonVariantConst value = object[key.c_str()];
    if (value.isNull())
        return fallback;
    if (value.is<bool>())
        return value.as<bool>();
    if (value.is<const char *>())
    {
        const String text = value.as<String>();
        if (text.equalsIgnoreCase("true") || text == "1" || text.equalsIgnoreCase("on"))
            return true;
        if (text.equalsIgnoreCase("false") || text == "0" || text.equalsIgnoreCase("off"))
            return false;
    }
    return value.as<int>() != 0;
}

String jsonString(JsonObjectConst object, const String &key, const String &fallback = String())
{
    const JsonVariantConst value = object[key.c_str()];
    return value.isNull() ? fallback : value.as<String>();
}

bool safeNtripField(const String &value)
{
    return value.indexOf('\r') < 0 && value.indexOf('\n') < 0;
}

struct LocalNtripConfig
{
    String host;
    String mountpoint;
    String password;
    uint16_t port = NTRIP_CASTER_PORT;
};

LocalNtripConfig loadLocalNtripConfig()
{
    LocalNtripConfig legacy;
    prefs.begin("myPrefs", true);
    legacy.host = prefs.getString("NTRIP_SERVER", String(NTRIP_CASTER_ADDRESS));
    legacy.port = prefs.getUShort("NTRIP_PORT", NTRIP_CASTER_PORT);
    legacy.mountpoint = prefs.getString("NTRIP_MPT", String(NTRIP_MOUNTPOINT));
    legacy.password = prefs.getString("NT_AUTH_BS", String(NTRIP_AUTH_BASE_STATION));
    prefs.end();
    return legacy;
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

String base64Encode(const String &plain)
{
    static constexpr char alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    String encoded;
    encoded.reserve(((plain.length() + 2) / 3) * 4);
    for (size_t i = 0; i < plain.length(); i += 3)
    {
        const uint32_t a = static_cast<uint8_t>(plain[i]);
        const uint32_t b = i + 1 < plain.length() ? static_cast<uint8_t>(plain[i + 1]) : 0;
        const uint32_t c = i + 2 < plain.length() ? static_cast<uint8_t>(plain[i + 2]) : 0;
        const uint32_t value = (a << 16) | (b << 8) | c;
        encoded += alphabet[(value >> 18) & 0x3F];
        encoded += alphabet[(value >> 12) & 0x3F];
        encoded += i + 1 < plain.length() ? alphabet[(value >> 6) & 0x3F] : '=';
        encoded += i + 2 < plain.length() ? alphabet[value & 0x3F] : '=';
    }
    return encoded;
}

namespace ntrip_common
{
    constexpr size_t CONFIG_DOCUMENT_SIZE = 4096;
    std::array<NtripEndpoint, NTRIP_SERVER_COUNT> endpoints;
    WiFiClient wifiNtripClient1;
    WiFiClient wifiNtripClient2;
    TinyGsmClient gsmNtripClient1(modem, 0);
    TinyGsmClient gsmNtripClient2(modem, 2);
    bool roverSessionActive = false;
    bool roverSessionAuthenticated = false;
    uint32_t roverLastGgaMs = 0;
    uint32_t roverBytesReceived = 0;
    uint32_t roverPacketsReceived = 0;

    Client &clientFor(uint8_t index)
    {
        if (index == 0)
        {
            return isWifiConnection()
                       ? static_cast<Client &>(wifiNtripClient1)
                       : static_cast<Client &>(gsmNtripClient1);
        }
        return isWifiConnection()
                   ? static_cast<Client &>(wifiNtripClient2)
                   : static_cast<Client &>(gsmNtripClient2);
    }

    void clearEndpointState(NtripEndpoint &endpoint)
    {
        endpoint.authenticated = false;
        endpoint.bytesInWindow = 0;
        endpoint.bytesPerSecond = 0;
        endpoint.windowStartedMs = millis();
        endpoint.lastKeepAliveMs = endpoint.windowStartedMs;
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

        const LocalNtripConfig legacy = loadLocalNtripConfig();

        for (uint8_t index = 0; index < NTRIP_SERVER_COUNT; ++index)
        {
            loadEndpoint(endpoints[index], index, service, hasServiceConfig, legacy);
        }
    }

    bool streamActive(uint8_t index)
    {
        if (index >= NTRIP_SERVER_COUNT || backendDeviceLocked(index + 1) || !backendLicenseValid() || !backendDeviceProvisioned() || !backendBaseStartupConfigured())
            return false;
        DynamicJsonDocument serviceDoc(CONFIG_DOCUMENT_SIZE);
        if (deserializeJson(serviceDoc, backendAgent.serviceConfigJson()) != DeserializationError::Ok || !serviceDoc.is<JsonObject>())
        {
            return index == 0;
        }
        return BackendAgent::effectiveStreamActive(
            serviceDoc.as<JsonObjectConst>(), index + 1, backendControlPlaneFailOpen());
    }

    void stopEndpoint(uint8_t index)
    {
        clientFor(index).stop();
        clearEndpointState(endpoints[index]);
    }

    bool waitForNtripResponse(Client &client, String *statusLine)
    {
        String response;
        response.reserve(512);
        const uint32_t deadline = millis() + 10000UL;
        bool headerComplete = false;
        bool readFailed = false;
        while ((client.connected() || client.available() > 0) && millis() < deadline && response.length() < 4096 && !headerComplete && !readFailed)
        {
            while (client.available() > 0 && response.length() < 4096 && !headerComplete && !readFailed)
            {
                const auto value = client.read();
                if (value < 0)
                {
                    readFailed = true;
                }
                else
                {
                    response += static_cast<char>(value);
                    headerComplete = response.endsWith("\r\n\r\n");
                }
            }
            if (!headerComplete && !readFailed)
                delay(1);
        }
        if (statusLine)
        {
            const auto end = response.indexOf('\n');
            *statusLine = response.substring(0, end >= 0 ? end : 240);
            statusLine->trim();
            if (statusLine->isEmpty())
                *statusLine = "No response from caster (timeout or closed connection)";
        }
        return response.indexOf("ICY 200 OK") >= 0 || response.indexOf("HTTP/1.1 200 OK") >= 0 || response.indexOf("HTTP/1.0 200 OK") >= 0;
    }

} // namespace ntrip_common
