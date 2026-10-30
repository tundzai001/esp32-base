#include "helper.h"
#include "functions/Backend_Agent.h"
#include "functions/RTCM_Receiver.h"
#include "functions/cmd_handler.h"
#include "functions/NTRIP_Handler_IP.h"

#include <time.h>
#include "esp_bt.h"

// ================= ĐỊNH NGHĨA CÁC BIẾN TOÀN CỤC =================
Preferences prefs;

extern PubSubClient mqtt;

namespace {
    constexpr uint32_t GNSS_DATA_TIMEOUT_MS = 5000;

    // The MQTT task also runs WebSocket and JSON command handling, so it
    // needs more stack than the serial console or periodic health task.
    constexpr uint32_t SERIAL_COMMAND_TASK_STACK_BYTES = 4096;
    constexpr uint32_t MQTT_TASK_STACK_BYTES = 12288;
    constexpr uint32_t NTRIP_TASK_STACK_BYTES = 6144;
    constexpr uint32_t HEALTH_TASK_STACK_BYTES = 4096;

    struct RtcmState {
        uint32_t lastGnssReceptionMs = 0;
        bool hasGnssReception = false;
        RtcmMessageCounts messageCounts;
    };

    class DeviceRuntime {
    public:
        inline static uint8_t mqttDisconnectCount = 0;
        inline static uint8_t ntripDisconnectCount = 0;
        inline static uint8_t gsmDisconnectCount = 0;
    };
}

SemaphoreHandle_t rtcmBufferMutex = nullptr;
SemaphoreHandle_t tcpStreamMutex = nullptr;
SemaphoreHandle_t gnssSerialMutex = nullptr;

void initPrefs();
static void serviceMqtt(const bool reconnect);
static void settleModemBeforeNtrip();
static bool tryWifiFallback(bool protocolsRunning);
static void checkSimInternetFallback();
static void checkSimRecovery();
static void activateNetwork(ConnectionType type, bool protocolsRunning);
static bool wifiFallbackActive = false;
static uint32_t lastSimProbeMs = 0;
static uint8_t simRecoverySuccesses = 0;

__attribute__((noreturn)) void taskNtrip([[maybe_unused]] void *parameter);
__attribute__((noreturn)) void healthCheckTask([[maybe_unused]] void *parameter);
__attribute__((noreturn)) void taskMQTT([[maybe_unused]] void *parameter);
__attribute__((noreturn)) void taskSerialCommand([[maybe_unused]] void *parameter);

void setup() {
    static RtcmState rtcmState;
    Serial.begin(115200);

    Serial.println("[DEBUG] Serial initialized");
    Serial.println("\n=========================================");
    Serial.println("     ESP32 GNSS GATEWAY KHOI DONG        ");
    Serial.println("=========================================");
    Serial.printf("[FIRMWARE] %s | build %s %s\n", AGENT_VERSION, __DATE__, __TIME__);

    // Release controller memory because this firmware does not use Bluetooth.
    esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);

    prefs.begin("myPrefs");
    const bool needReset = prefs.getBool("NEED_RESET", true);
    const int restartCount = prefs.getInt("RSTRT_COUNT", 0);
    prefs.putInt("RSTRT_COUNT", restartCount + 1);
    if (needReset) {
        Serial.println("[SETUP] Khoi tao Preferences lan dau tien...");
        initPrefs();
    }

    loadConnectionTypeFromPrefs();
    const int gnssTX = TX_GNSS;
    const int gnssRX = RX_GNSS;
    const int rx2ModemTX = RX_TO_MODEM_TX;
    const int tx2ModemRX = TX_TO_MODEM_RX;
    prefs.end();

    (void)restartCount;
    Serial.println("[SETUP] Khoi dong UART GNSS: TX=" + String(gnssTX)
                   + ", RX=" + String(gnssRX));
    // Buffer GNSS bursts while network tasks wait on sockets.
    Serial1.setRxBufferSize(8192);
    Serial1.begin(GNSS_BAUD, SERIAL_8N1, static_cast<uint8_t>(gnssRX), static_cast<uint8_t>(gnssTX));
    Serial1.setTimeout(100);

    // Restore identity, activation and services before any transport can publish.
    backendAgent.begin();

    const GnssCmdBuilder::GnssSensorType detectedSensorType = detectGnssSensorType();
    const char *detectedSensorTypeName = "INVALID";
    if (detectedSensorType == GnssCmdBuilder::GnssSensorType::Ublox) {
        detectedSensorTypeName = "UBLOX";
        backendAgent.setDetectedChipIfUnknown("Ublox");
    } else if (detectedSensorType == GnssCmdBuilder::GnssSensorType::Unicore) {
        detectedSensorTypeName = "UNICORE";
        backendAgent.setDetectedChipIfUnknown("Unicorecomm");
    }
    prefs.begin("myPrefs", false);
    prefs.putString("GNSS_MDL_TYPE", detectedSensorTypeName);
    prefs.end();

    Serial.println("[SETUP] Da nhan dien chip GNSS: " + String(detectedSensorTypeName));

    rtcmBufferMutex = xSemaphoreCreateMutex();
    while (rtcmBufferMutex == nullptr) {
        Serial.println("[ERROR] Tao mutex rtcmDataMutex that bai! Dang thu lai...");
        rtcmBufferMutex = xSemaphoreCreateMutex();
    }
    Serial.println("[SETUP] Tao mutex rtcmDataMutex thanh cong!");

    tcpStreamMutex = xSemaphoreCreateMutex();
    while (tcpStreamMutex == nullptr) {
        Serial.println("[ERROR] Tao mutex tcpStreamMutex that bai! Dang thu lai...");
        tcpStreamMutex = xSemaphoreCreateMutex();
    }
    Serial.println("[SETUP] Tao mutex tcpStreamMutex thanh cong!");

    gnssSerialMutex = xSemaphoreCreateMutex();
    while (gnssSerialMutex == nullptr) {
        Serial.println("[ERROR] Tao mutex gnssSerialMutex that bai! Dang thu lai...");
        gnssSerialMutex = xSemaphoreCreateMutex();
    }
    Serial.println("[SETUP] Tao mutex gnssSerialMutex thanh cong!");

    backendAgent.initializeBaseAtBoot(detectedSensorType);

    // Keep the local console responsive even if Wi-Fi/GSM cannot connect.
    Serial.println("[SETUP] Starting Serial Command task before network setup.");
    xTaskCreatePinnedToCore(taskSerialCommand, "Serial CMD Task", SERIAL_COMMAND_TASK_STACK_BYTES,
                            nullptr, 1, nullptr, 1);

#ifndef NATIVE_BUILD
    if (isGsmConnection()) {
        SerialAT.begin(115200, SERIAL_8N1,
                       static_cast<uint8_t>(rx2ModemTX), static_cast<uint8_t>(tx2ModemRX));
        delay(500);
    }
#endif

    bool networkConnected = false;
    while (!networkConnected) {
        if (isWifiConnection()) {
            Serial.println("[SETUP] Su dung ket noi WIFI");
            networkConnected = setupWiFi();
        } else {
            Serial.println("[SETUP] Su dung ket noi SIM/GSM");
            DeviceRuntime::gsmDisconnectCount = 0;
            networkConnected = startSIM() && connectGSM(1);
            if (!networkConnected) networkConnected = tryWifiFallback(false);
        }
        if (networkConnected) {
            Serial.println("[SETUP] Ket noi mang thanh cong!");
            setupNTRIP();
            DeviceRuntime::ntripDisconnectCount = 0;
            if (isGsmConnection()) {
                settleModemBeforeNtrip();
            }
            connectNTRIP();
            DeviceRuntime::mqttDisconnectCount = 0;
            setupMQTT();
        } else {
            Serial.println("[ERROR] Khong the ket noi mang. Vui long kiem tra cau hinh va thu lai.");
        }
    }

    Serial.println("[SETUP] Ket noi mang thanh cong!");
    configTime(0, 0, "pool.ntp.org", "time.google.com");

    Serial.println("[SETUP] Task MQTT: Quan ly ket noi MQTT va callback.");
    xTaskCreatePinnedToCore(taskMQTT, "MQTT Task", MQTT_TASK_STACK_BYTES, nullptr, 3, nullptr, 1);
    Serial.println("[SETUP] Da khoi dong Task MQTT!");

    Serial.println("[SETUP] Task NTRIP: Gui du lieu RTCM qua NTRIP.");
    xTaskCreatePinnedToCore(taskNtrip, "NTRIP Task", NTRIP_TASK_STACK_BYTES, &rtcmState, 2, nullptr, 1);
    Serial.println("[SETUP] Da khoi dong Task NTRIP!");

    Serial.println("[SETUP] Task Health: Gui thong tin suc khoe thiet bi len MQTT moi 30s");
    xTaskCreatePinnedToCore(healthCheckTask, "Health Task", HEALTH_TASK_STACK_BYTES,
                            &rtcmState, 1, nullptr, 1);
    Serial.println("[SETUP] Da khoi dong Task Health!");

    Serial.println("=========================================");
    Serial.println("        KHOI DONG HOAN TAT               ");
    Serial.println("=========================================\n");
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
}

__attribute__((noreturn)) void taskNtrip([[maybe_unused]] void *parameter) {
    auto* const rtcmState = static_cast<RtcmState*>(parameter);
    Serial.println("[NTRIP TASK] Bat dau task NTRIP...");
    int loopStatus = 0;
    while (true) {
        String rtcmRead;
        if (rtcmBufferMutex && xSemaphoreTake(rtcmBufferMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS))) {
            rtcmRead = receiveRtcmFromGnss();
            if (!rtcmRead.isEmpty()) {
                rtcmState->lastGnssReceptionMs = millis();
                rtcmState->hasGnssReception = true;
            }
            xSemaphoreGive(rtcmBufferMutex);
        }

        if (isGsmConnection()) {
            if (!xSemaphoreTake(tcpStreamMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS))) {
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }
            const bool gprsConnected = modem.isGprsConnected();
            xSemaphoreGive(tcpStreamMutex);
            if (!gprsConnected) {
                // The network manager retries SIM or switches to Wi-Fi. Keep
                // GNSS and the local console alive while connectivity recovers.
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
            DeviceRuntime::gsmDisconnectCount = 0;
        }
        if (xSemaphoreTake(tcpStreamMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS))) {
            // loopNTRIP owns reconnect attempts and the per-endpoint backoff.
            loopStatus = loopNTRIP(rtcmRead);
            xSemaphoreGive(tcpStreamMutex);
            // A caster rejection is not a hardware failure. loopNTRIP keeps
            // its per-endpoint reconnect backoff while MQTT remains available.
            #if PROGRAM_DEBUG
            Serial.println("[NTRIP TASK] loopNTRIP() tra ve: " + String(loopStatus));
            #endif
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
__attribute__((noreturn)) void healthCheckTask([[maybe_unused]] void* const parameter) {
    auto* const rtcmState = static_cast<RtcmState*>(parameter);
    // Read the shared GNSS snapshot under its mutex before checking network health.
    String healthPayload = "";
    uint32_t loopStartTime = 0;
    while (true) {
        const uint32_t loopStartTime = millis();
        healthPayload = "";
        int32_t signalQualityDbm = -1;
        if (xSemaphoreTake(rtcmBufferMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)))
        {
            const bool gnssDataOk = rtcmState->hasGnssReception
                && millis() - rtcmState->lastGnssReceptionMs <= GNSS_DATA_TIMEOUT_MS;
            const RtcmMessageCounts messageCounts = rtcmState->messageCounts;
            rtcmState->messageCounts = {};
            xSemaphoreGive(rtcmBufferMutex);
            // Both modem RSSI and client connected() checks issue AT commands.
            if (xSemaphoreTake(tcpStreamMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS))) {
                if (isGsmConnection()) signalQualityDbm = modem.getSignalQuality();
                healthPayload = formDeviceHealthString(signalQualityDbm, gnssDataOk,
                                                       messageCounts.values);
                xSemaphoreGive(tcpStreamMutex);
            }
        }
        if (!healthPayload.isEmpty()) publishHealth(healthPayload);
        const uint32_t elapsed = millis() - loopStartTime;
        vTaskDelay(pdMS_TO_TICKS(elapsed < HEALTH_INTERVAL ? HEALTH_INTERVAL - elapsed : 1));
    }
}

__attribute__((noreturn)) void taskMQTT([[maybe_unused]] void *parameter) {
    Serial.println("[MQTT TASK] Bat dau task MQTT...");
    while (true) {
        if (tcpStreamMutex && xSemaphoreTake(tcpStreamMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS))) {
            checkSimInternetFallback();
            serviceMqtt(true);
            checkSimInternetFallback();
            backendAgent.loop();
            xSemaphoreGive(tcpStreamMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

__attribute__((noreturn)) void taskSerialCommand([[maybe_unused]] void *parameter) {
    SerialCommandProcessor commandProcessor;
    Serial.println("[SERIAL COMMAND TASK] Bat dau task lang nghe Serial...");
    while (true) {
        commandProcessor.processPending();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void loop() {
    if (isGsmConnection() && tcpStreamMutex
        && xSemaphoreTake(tcpStreamMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS))) {
        if (!modem.isGprsConnected()) {
            digitalWrite(LED_PIN, HIGH);
#if RTCM_COMMUNICATION_PROTOCOL == TCP_IP
            stopNtrip();
#endif
            if (tryWifiFallback(true)) {
                DeviceRuntime::gsmDisconnectCount = 0;
            } else if (connectGSM(1)) {
                DeviceRuntime::gsmDisconnectCount = 0;
#if RTCM_COMMUNICATION_PROTOCOL == TCP_IP
                setupNTRIP();
                connectNTRIP();
#endif
            } else {
                ++DeviceRuntime::gsmDisconnectCount;
            }
        } else {
            DeviceRuntime::gsmDisconnectCount = 0;
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
            setupWiFi();
            digitalWrite(LED_PIN, LOW);
        }
        xSemaphoreGive(tcpStreamMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
}

// Called with the shared modem mutex held once worker tasks are running.
static bool tryWifiFallback(bool protocolsRunning) {
    Serial.println("[NETWORK] SIM khong truy cap duoc dich vu; thu Wi-Fi du phong da luu.");
    if (!setupWiFi()) return false;
    const bool reachable = probeMqttTransport(false);
    if (!reachable) {
        Serial.println("[NETWORK] Wi-Fi chua truy cap duoc MQTT broker; giu SIM va se thu lai.");
        WiFi.disconnect();
        return false;
    }
    activateNetwork(ConnectionType::WIFI, protocolsRunning);
    wifiFallbackActive = true;
    lastSimProbeMs = millis();
    simRecoverySuccesses = 0;
    Serial.println("[NETWORK] Da chuyen sang Wi-Fi du phong; giu nguyen tuy chon SIM da luu.");
    return true;
}

// Both switching directions close old sockets before selecting new clients.
static void activateNetwork(ConnectionType type, bool protocolsRunning) {
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

// Only Wi-Fi entered through SIM fallback is eligible for automatic return.
static void checkSimRecovery() {
    if (!wifiFallbackActive || !isWifiConnection()) return;
    const bool wifiConnected = WiFiClass::status() == WL_CONNECTED;
    const uint32_t now = millis();
    const uint32_t interval = wifiConnected ? 60000UL : 15000UL;
    if (now - lastSimProbeMs < interval) return;
    lastSimProbeMs = now;
    // Avoid a blocking registration wait while healthy Wi-Fi carries traffic.
    const bool reachable = modem.isNetworkConnected()
        && (modem.isGprsConnected() || connectGSM(1))
        && probeMqttTransport(true);
    if (!reachable) {
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
}

static void checkSimInternetFallback() {
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

static void settleModemBeforeNtrip() {
    Serial.println("[SETUP][NTRIP] Cho modem on dinh truoc khi bat tay NTRIP...");
    const uint32_t settleStart = millis();
    const uint32_t settleDurationMs = 600;

    while (millis() - settleStart < settleDurationMs) {
        modem.maintain();
        delay(20);
    }

    Serial.println("[SETUP][NTRIP] Modem da on dinh, bat dau ket noi NTRIP.");
}

void initPrefs() {
    prefs.clear();
    prefs.putBool("NEED_RESET", false);
    prefs.putInt("RSTRT_COUNT", 0);
    // Hai giá trị hợp lệ: "4G" và "WIFI". Giá trị này được nạp khi khởi động.
    prefs.putString("CONNECTION_TYPE", "4G");
    prefs.putString("GNSS_MDL_TYPE", "INVALID");

    prefs.putString("APN", APN);
    prefs.putString("WIFI_SSID", WIFI_SSID);
    prefs.putString("WIFI_PASS", WIFI_PASSWORD);
    prefs.putString("GPRS_USER", "");
    prefs.putString("GPRS_PASS", "");
    prefs.putString("NTRIP_SERVER", NTRIP_CASTER_ADDRESS);
    prefs.putUShort("NTRIP_PORT", NTRIP_CASTER_PORT);
    prefs.putString("NTRIP_MPT", NTRIP_MOUNTPOINT);
    prefs.putString("NT_AUTH_BS", NTRIP_AUTH_BASE_STATION);

    prefs.putString("MQTT_SERVER", MQTT_SERVER);
    prefs.putUShort("MQTT_PORT", MQTT_PORT);
    prefs.putString("MQTT_USER", MQTT_USER);
    prefs.putString("MQTT_PASS", MQTT_PASS);

    prefs.putString("TPC_SUB_CMD", TOPIC_SUB_CMD);
    prefs.putString("TPC_HEALTH", TOPIC_PUB_HEALTH);
}

static void serviceMqtt(const bool reconnect) {
    if (mqtt.connected()) {
        DeviceRuntime::mqttDisconnectCount = 0;
        mqtt.loop();
        return;
    }

    backendAgent.onMqttDisconnected();
    if (reconnect) {
        if (DeviceRuntime::mqttDisconnectCount < 255) ++DeviceRuntime::mqttDisconnectCount;
        connectMQTT();
    }
}
