#ifndef NTRIP_COMMON_H
#define NTRIP_COMMON_H

#include <Arduino.h>
#include <array>
#include <cstdint>

#include "Top_Lvl_Config.h"
#include "hardware/Connection_type.h"

extern SemaphoreHandle_t rtcmBufferMutex;
extern SemaphoreHandle_t tcpStreamMutex;

bool safeNtripField(const String &value);
String base64Encode(const String &plain);

namespace ntrip_common
{

    inline constexpr uint8_t NTRIP_SERVER_COUNT = 2;
    inline constexpr uint32_t RECONNECT_INTERVAL_MS = 5000;
    inline constexpr uint32_t BPS_WINDOW_MS = 1000;

    struct NtripEndpoint
    {
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

    extern std::array<NtripEndpoint, NTRIP_SERVER_COUNT> endpoints;
    extern bool roverSessionActive;
    extern bool roverSessionAuthenticated;
    extern uint32_t roverLastGgaMs;
    extern uint32_t roverBytesReceived;
    extern uint32_t roverPacketsReceived;

    Client &clientFor(uint8_t index);
    void clearEndpointState(NtripEndpoint &endpoint);
    void loadEndpoints();
    bool streamActive(uint8_t index);
    void stopEndpoint(uint8_t index);
    bool waitForNtripResponse(Client &client, String *statusLine = nullptr);
    void updateBps(NtripEndpoint &endpoint);
    bool sendEndpointRtcm(NtripEndpoint &endpoint, Client &client, const String &rtcmData);

} // namespace ntrip_common

#endif
