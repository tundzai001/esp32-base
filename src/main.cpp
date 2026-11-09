#include "helper.h"
#include "functions/backend/Backend_Agent.h"
#include "functions/ntrip/RTCM_Receiver.h"
#include "functions/system/cmd_handler.h"
#include "functions/ntrip/ntrip_base.h"
#include "functions/ntrip/ntrip_rover.h"
#include "functions/gnss/Gnss_Detection.h"
#include "hardware/Network_Manager.h"

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

}

SemaphoreHandle_t rtcmBufferMutex = nullptr;
SemaphoreHandle_t tcpStreamMutex = nullptr;
SemaphoreHandle_t gnssSerialMutex = nullptr;

void initPrefs();
static void serviceMqtt();

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
#if PROGRAM_DEBUG
    Serial.println("[DEBUG] PROGRAM_DEBUG=1");
#endif

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

    NetworkManager::connect();
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
#if PROGRAM_DEBUG
    bool ntripStatusLogged = false;
    bool ntripWasConnected = false;
#endif
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
        }
        if (xSemaphoreTake(tcpStreamMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS))) {
            // loopNTRIP owns reconnect attempts and the per-endpoint backoff.
            loopStatus = loopNTRIP(rtcmRead);
            xSemaphoreGive(tcpStreamMutex);
            // A caster rejection is not a hardware failure. loopNTRIP keeps
            // its per-endpoint reconnect backoff while MQTT remains available.
            #if PROGRAM_DEBUG
            const bool connected = loopStatus != 504;
            if (!ntripStatusLogged || connected != ntripWasConnected) {
                Serial.printf("[NTRIP TASK] NTRIP=%s; loop_status=%d\n",
                              connected ? "UP" : "DOWN", loopStatus);
                ntripStatusLogged = true;
                ntripWasConnected = connected;
            }
            #endif
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
__attribute__((noreturn)) void healthCheckTask([[maybe_unused]] void* const parameter) {
    auto* const rtcmState = static_cast<RtcmState*>(parameter);
    // Read the shared GNSS snapshot under its mutex before checking network health.
    String healthPayload = "";
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
                                                       messageCounts.values.data());
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
            NetworkManager::checkInternetFallback();
            serviceMqtt();
            NetworkManager::checkInternetFallback();
            backendAgent.loop();
            NetworkManager::logStatus();
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
    NetworkManager::maintain();
    vTaskDelay(pdMS_TO_TICKS(1000));
}


void initPrefs() {
    prefs.clear();
    prefs.putBool("NEED_RESET", false);
    prefs.putInt("RSTRT_COUNT", 0);
    // Hai giá trị hợp lệ: "4G" và "WIFI". Giá trị này được nạp khi khởi động.
    prefs.putString("CONNECTION_TYPE", CONNECTION_PREFERENCE);
    prefs.putString("GNSS_MDL_TYPE", "INVALID");

    prefs.putString("APN", APN);
    prefs.putString("WIFI_SSID", WIFI_SSID);
    prefs.putString("WIFI_PASS", WIFI_PASSWORD);
    prefs.putString("GPRS_USER", GPRS_USER);
    prefs.putString("GPRS_PASS", GPRS_PASS);
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

static void serviceMqtt() {
    if (mqtt.connected()) {
        mqtt.loop();
        return;
    }

    backendAgent.onMqttDisconnected();
    connectMQTT();
}
