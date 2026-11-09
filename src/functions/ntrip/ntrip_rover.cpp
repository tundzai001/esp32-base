#include "functions/ntrip/ntrip_rover.h"
#include "Prog_Config.h"
#include "functions/backend/Backend_Agent_Codec.h"
#include "functions/ntrip/RTCM_Receiver.h"
#include <array>

using BackendAgentInternal::encodeBase64;
using namespace ntrip_common;

namespace
{
    Vn2000Session roverDatumSession;
    uint32_t roverBytesReceived = 0;
    uint32_t roverPacketsReceived = 0;
    uint32_t roverBytesWritten = 0;
    uint32_t roverLastDiagnosticMs = 0;
    const char *roverLastError = "CORS rover connection lost";

    void sendRoverGgaIfAvailable(Client &client, const bool force = false)
    {
        if (!force && millis() - roverLastGgaMs < 5000UL)
            return;
        const GnssTelemetrySnapshot snapshot = getGnssTelemetrySnapshot();
        if (snapshot.lastGga.front() == '\0' || snapshot.lastGgaMs == 0 || millis() - snapshot.lastGgaMs > 3000UL)
        {
            if (force || millis() - roverLastGgaMs >= 5000UL)
            {
                Serial.println("[NTRIP ROVER] Chua co GGA moi tu UART GNSS; chua gui GGA len caster");
                roverLastGgaMs = millis();
            }
            return;
        }

        if (const String outbound = String(snapshot.lastGga.data()) + "\r\n";
            client.print(outbound) != outbound.length())
        {
            Serial.println("[NTRIP ROVER] Loi ghi GGA len socket caster");
            roverLastGgaMs = millis();
            return;
        }
        Serial.printf("[NTRIP ROVER] GGA -> caster: %s\n", snapshot.lastGga.data());
        roverLastGgaMs = millis();
    }

} // namespace

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
    roverDatumSession.reset();
    roverBytesReceived = 0;
    roverPacketsReceived = 0;
    roverBytesWritten = 0;
    roverLastDiagnosticMs = millis();
    roverLastError = "CORS rover connection lost";
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

    const String user = username.isEmpty() ? String("source") : username;
    const String auth = encodeBase64(user + ":" + password);
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

    if (!waitForNtripResponse(client, nullptr, true))
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
        Serial.printf("[NTRIP ROVER][ERROR] CORS socket closed; cors_rx=%lu B; uart_tx=%lu B; rtcm_frames=%lu\n",
                      static_cast<unsigned long>(roverBytesReceived),
                      static_cast<unsigned long>(roverBytesWritten),
                      static_cast<unsigned long>(roverDatumSession.validFrames()));
        roverSessionActive = false;
        roverSessionAuthenticated = false;
        return 504;
    }
    sendRoverGgaIfAvailable(client);
    if (millis() - roverLastDiagnosticMs >= 5000UL)
    {
        roverLastDiagnosticMs = millis();
        const auto snapshot = getGnssTelemetrySnapshot();
        Serial.printf("[NTRIP ROVER][DATA] cors_rx=%lu B; uart_tx=%lu B; rtcm_frames=%lu; fix=%s\n",
                      static_cast<unsigned long>(roverBytesReceived),
                      static_cast<unsigned long>(roverBytesWritten),
                      static_cast<unsigned long>(roverDatumSession.validFrames()), snapshot.fixStatus.data());
    }
    if (client.available() <= 0)
        return 200;

    std::array<uint8_t, 512> buffer{};
    if (!rtcmBufferMutex || xSemaphoreTake(rtcmBufferMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) != pdTRUE)
    {
        return 200;
    }
    const int available = client.available();
    const size_t bytesToRead = min(static_cast<size_t>(available), buffer.size());
    if (const int bytesRead = client.read(buffer.data(), bytesToRead); bytesRead > 0)
    {
        roverDatumSession.feed(buffer.data(), static_cast<size_t>(bytesRead));
        const size_t written = Serial1.write(buffer.data(), static_cast<size_t>(bytesRead));
        roverBytesReceived += static_cast<uint32_t>(bytesRead);
        roverBytesWritten += static_cast<uint32_t>(written);
        ++roverPacketsReceived;
        if (written != static_cast<size_t>(bytesRead))
        {
            roverLastError = "GNSS UART did not accept all CORS bytes";
            Serial.printf("[NTRIP ROVER][ERROR] UART accepted %u/%u bytes\n",
                          static_cast<unsigned>(written), static_cast<unsigned>(bytesRead));
            xSemaphoreGive(rtcmBufferMutex);
            return 504;
        }
    }
    xSemaphoreGive(rtcmBufferMutex);
    // Return status, not byte count: a valid 504-byte read must not look like failure.
    return 200;
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

Vn2000Datum ntripRoverDatum() { return roverDatumSession.datum(); }

const char *ntripRoverLastError() { return roverLastError; }
