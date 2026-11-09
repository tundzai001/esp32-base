#include "hardware/Network_Manager.h"
#include "helper.h"
#include "functions/backend/Backend_Agent.h"

namespace {
bool tryWifiFallback(bool protocolsRunning);
void activateNetwork(ConnectionType type, bool protocolsRunning);
void checkSimRecovery();
void checkSimInternetFallback();
void settleModemBeforeNtrip();
void logNetworkStatus(bool force = false);
bool wifiFallbackActive = false;
uint32_t lastSimProbeMs = 0;
uint8_t simRecoverySuccesses = 0;

// Called with the shared modem mutex held once worker tasks are running.
bool tryWifiFallback(bool protocolsRunning) {
    const bool simPreferred = isGsmConnection() || wifiFallbackActive;
    Serial.println("[NETWORK] Dang thu cac mang Wi-Fi da luu.");
    // setupWiFi disconnects STA before trying profiles. Release the caster's
    // source session while the old Wi-Fi route can still deliver TCP FIN.
    // Keep SIM sockets alive until a Wi-Fi candidate passes validation.
    if (protocolsRunning && isWifiConnection()) {
        mqtt.disconnect();
        stopNtrip();
    }
    if (!setupWiFi([] { return probeMqttTransport(false); })) return false;
    activateNetwork(ConnectionType::WIFI, protocolsRunning);
    wifiFallbackActive = simPreferred;
    lastSimProbeMs = millis();
    simRecoverySuccesses = 0;
    Serial.println("[NETWORK] Da ket noi Wi-Fi; giu nguyen tuy chon mang da luu.");
    logNetworkStatus(true);
    return true;
}

// Both switching directions close old sockets before selecting new clients.
void activateNetwork(ConnectionType type, bool protocolsRunning) {
    if (protocolsRunning) {
        mqtt.disconnect();
        stopNtrip();
    }
    connectionType = type;
    backendAgent.onNetworkChanged();
    if (protocolsRunning) {
        setupMQTT();
        setupNTRIP();
        connectNTRIP();
    }
}

// Caller holds tcpStreamMutex after startup; connected() may issue modem AT.
void logNetworkStatus(bool force) {
    static uint32_t lastLogMs = 0;
    const uint32_t now = millis();
    if (!force && now - lastLogMs < 30000UL) return;
    lastLogMs = now;
    Serial.printf("[NETWORK][ACTIVE] transport=%s; fallback=%s; MQTT=%s; NTRIP=%s\n",
                  isWifiConnection() ? "WIFI" : "SIM/4G",
                  wifiFallbackActive ? "YES" : "NO",
                  mqtt.connected() ? "UP" : "DOWN",
                  isNtripConnected() ? "UP" : "DOWN");
    if (isWifiConnection()) {
        Serial.printf("[NETWORK][WIFI] connected=%s; SSID=%s; IP=%s; RSSI=%d dBm\n",
                      WiFiClass::status() == WL_CONNECTED ? "YES" : "NO",
                      WiFi.SSID().c_str(), WiFi.localIP().toString().c_str(), WiFi.RSSI());
    }
}

// Only Wi-Fi entered through SIM fallback is eligible for automatic return.
void checkSimRecovery() {
    if (!wifiFallbackActive || !isWifiConnection()) return;
    const bool wifiConnected = WiFiClass::status() == WL_CONNECTED;
    const uint32_t now = millis();
    
    if (const uint32_t interval = wifiConnected ? 60000UL : 15000UL;
            now - lastSimProbeMs < interval
         ) return;
    lastSimProbeMs = now;
    // Avoid a blocking registration wait while healthy Wi-Fi carries traffic.
    
    if (const bool reachable = modem.isNetworkConnected()
        && (modem.isGprsConnected() || connectGSM(1))
        && probeMqttTransport(true);
        !reachable) 
    {
        simRecoverySuccesses = 0;
        return;
    }
    // Two successful broker handshakes a minute apart avoid switching on a blip.
    if (++simRecoverySuccesses < 2 && wifiConnected) return;
    Serial.println("[NETWORK] SIM da truy cap lai MQTT broker; chuyen ve SIM.");
    activateNetwork(ConnectionType::GSM, true);
    wifiFallbackActive = false;
    simRecoverySuccesses = 0;
    WiFi.disconnect();
    logNetworkStatus(true);
}

void checkSimInternetFallback() {
    static uint32_t unavailableSinceMs = 0;
    static uint32_t lastWifiAttemptMs = 0;
    if (!isGsmConnection()) return;
    if (mqtt.connected() || isNtripConnected()) {
        unavailableSinceMs = 0;
        return;
    }
    const uint32_t now = millis();
    if (unavailableSinceMs == 0) {
        unavailableSinceMs = now ? now : 1;
        Serial.println("[NETWORK] GPRS co the van ket noi nhung MQTT/NTRIP chua truy cap duoc; se thu Wi-Fi sau 30s.");
        return;
    }
    if (now - unavailableSinceMs < 30000UL) return;
    if (lastWifiAttemptMs != 0 && now - lastWifiAttemptMs < 30000UL) return;
    lastWifiAttemptMs = now;
    if (tryWifiFallback(true)) unavailableSinceMs = 0;
}

void settleModemBeforeNtrip() {
    Serial.println("[SETUP][NTRIP] Cho modem on dinh truoc khi bat tay NTRIP...");
    const uint32_t settleStart = millis();
    const uint32_t settleDurationMs = 600;

    while (millis() - settleStart < settleDurationMs) {
        modem.maintain();
        delay(20);
    }

    Serial.println("[SETUP][NTRIP] Modem da on dinh, bat dau ket noi NTRIP.");
}

} // namespace

namespace NetworkManager {

void connect() {
    bool networkConnected = false;
    while (!networkConnected) {
        if (isWifiConnection()) {
            Serial.println("[SETUP] Su dung ket noi WIFI");
            networkConnected = setupWiFi();
        } else {
            Serial.println("[SETUP] Su dung ket noi SIM/GSM");
            networkConnected = startSIM() && connectGSM(1);
            if (!networkConnected) networkConnected = tryWifiFallback(false);
        }
        if (networkConnected) {
            Serial.println("[SETUP] Ket noi mang thanh cong!");
            setupNTRIP();
            if (isGsmConnection()) {
                settleModemBeforeNtrip();
            }
            connectNTRIP();
            setupMQTT();
        } else {
            Serial.println("[ERROR] Khong the ket noi mang. Vui long kiem tra cau hinh va thu lai.");
            delay(1000);
        }
    }

    Serial.println("[SETUP] Ket noi mang thanh cong!");
    logNetworkStatus(true);
}

void maintain() {
    if (isGsmConnection() && tcpStreamMutex
        && xSemaphoreTake(tcpStreamMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS))) {
        if (!modem.isGprsConnected()) {
            digitalWrite(LED_PIN, HIGH);
#if RTCM_COMMUNICATION_PROTOCOL == TCP_IP
            stopNtrip();
#endif
            if (!tryWifiFallback(true) && connectGSM(1)) {
#if RTCM_COMMUNICATION_PROTOCOL == TCP_IP
                setupNTRIP();
                connectNTRIP();
#endif
            }
        } else {
            digitalWrite(LED_PIN, LOW);
            checkSimInternetFallback();
        }
        xSemaphoreGive(tcpStreamMutex);
    }

    if (isWifiConnection() && tcpStreamMutex
        && xSemaphoreTake(tcpStreamMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS))) {
        checkSimRecovery();
        if (isWifiConnection() && WiFiClass::status() != WL_CONNECTED) {
            digitalWrite(LED_PIN, HIGH);
            mqtt.disconnect();
            stopNtrip();
            setupWiFi();
            digitalWrite(LED_PIN, LOW);
        }
        xSemaphoreGive(tcpStreamMutex);
    }
}

void checkInternetFallback() { checkSimInternetFallback(); }
void logStatus() { logNetworkStatus(); }

} // namespace NetworkManager
