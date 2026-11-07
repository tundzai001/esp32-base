#define NTRIP_HANDLER_IP_CODE

#include "functions/ntrip/ntrip_rover.h"
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

using namespace ntrip_common;

void sendRoverGgaIfAvailable(Client &client, const bool force = false)
{
    if (!force && millis() - roverLastGgaMs < 5000UL)
        return;
    const GnssTelemetrySnapshot snapshot = getGnssTelemetrySnapshot();
    if (snapshot.lastGga[0] == '\0' || snapshot.lastGgaMs == 0 || millis() - snapshot.lastGgaMs > 3000UL)
    {
        if (force || millis() - roverLastGgaMs >= 5000UL)
        {
            Serial.println("[NTRIP ROVER] Chua co GGA moi tu UART GNSS; chua gui GGA len caster");
            roverLastGgaMs = millis();
        }
        return;
    }
    const auto outbound = String(snapshot.lastGga.data()) + "\r\n";
    if (client.print(outbound) != outbound.length())
    {
        Serial.println("[NTRIP ROVER] Loi ghi GGA len socket caster");
        roverLastGgaMs = millis();
        return;
    }
    Serial.printf("[NTRIP ROVER] GGA -> caster: %s\n", snapshot.lastGga.data());
    roverLastGgaMs = millis();
}

bool startNtripRover(const String &host, const uint16_t port, const String &username,
                     const String &password, String mountpoint, const uint8_t version)
{
    if (host.isEmpty() || port == 0 || mountpoint.isEmpty() || !safeNtripField(host) || !safeNtripField(username) || !safeNtripField(password) || !safeNtripField(mountpoint))
        return false;
    while (mountpoint.startsWith("/"))
        mountpoint.remove(0, 1);
    if (mountpoint.isEmpty())
        return false;

    // The temporary rover reuses source server 1's client and modem mux 0.
    // Close that client before opening a receive-only correction stream.
    roverBytesReceived = 0;
    roverPacketsReceived = 0;
    roverSessionActive = true;
    roverSessionAuthenticated = false;
    roverLastGgaMs = 0;
    Client &client = clientFor(0);
    client.stop();
    client.setTimeout(3000);
    if (!client.connect(host.c_str(), port))
    {
        roverSessionActive = false;
        return false;
    }

    const auto user = username.isEmpty() ? String("source") : username;
    const auto auth = base64Encode(user + ":" + password);
    String request;
    if (version == 2)
    {
        request = "GET /" + mountpoint + " HTTP/1.1\r\n"
                                         "Host: " +
                  host + ":" + String(port) + "\r\n"
                                              "Ntrip-Version: Ntrip/2.0\r\n"
                                              "User-Agent: NTRIP ESP32-Agent/" +
                  String(AGENT_VERSION) + "\r\n"
                                          "Authorization: Basic " +
                  auth + "\r\n"
                         "Accept: */*\r\n"
                         "Connection: Keep-Alive\r\n\r\n";
    }
    else
    {
        request = "GET /" + mountpoint + " HTTP/1.0\r\n"
                                         "User-Agent: NTRIP ESP32-Agent/" +
                  String(AGENT_VERSION) + "\r\n"
                                          "Authorization: Basic " +
                  auth + "\r\n"
                         "Accept: */*\r\n"
                         "Connection: close\r\n\r\n";
    }
    client.print(request);

    if (!waitForNtripResponse(client))
    {
        client.stop();
        roverSessionActive = false;
        return false;
    }
    roverSessionAuthenticated = true;
    sendRoverGgaIfAvailable(client, true);
    Serial.println("[NTRIP ROVER] Da ket noi CORS " + host + ":" + String(port) + "/" + mountpoint);
    return true;
}

int loopNtripRover()
{
    if (!roverSessionActive || !roverSessionAuthenticated)
        return 504;
    Client &client = clientFor(0);
    if (!client.connected())
    {
        roverSessionActive = false;
        roverSessionAuthenticated = false;
        return 504;
    }
    sendRoverGgaIfAvailable(client);
    if (client.available() <= 0)
        return 200;

    std::array<uint8_t, 512> buffer{};
    if (!rtcmBufferMutex || xSemaphoreTake(rtcmBufferMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) != pdTRUE)
    {
        return 200;
    }
    const auto available = client.available();
    const auto bytesToRead = min(static_cast<size_t>(available), sizeof(buffer));
    const auto bytesRead = client.read(buffer.data(), bytesToRead);
    if (bytesRead > 0)
    {
        Serial1.write(buffer.data(), static_cast<size_t>(bytesRead));
        roverBytesReceived += static_cast<uint32_t>(bytesRead);
        ++roverPacketsReceived;
    }
    xSemaphoreGive(rtcmBufferMutex);
    return bytesRead > 0 ? bytesRead : 200;
}

void stopNtripRover()
{
    if (roverSessionActive)
        clientFor(0).stop();
    roverSessionActive = false;
    roverSessionAuthenticated = false;
    roverLastGgaMs = 0;
}

bool ntripRoverActive()
{
    return roverSessionActive && roverSessionAuthenticated;
}

uint32_t ntripRoverBytesReceived() { return roverBytesReceived; }
uint32_t ntripRoverPacketsReceived() { return roverPacketsReceived; }
