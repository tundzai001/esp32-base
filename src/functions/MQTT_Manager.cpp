#include "functions/MQTT_Manager.h"
#include "Top_Lvl_Config.h"
#include "Prog_Config.h"
#include "functions/Backend_Agent.h"
#include "functions/cmd_handler.h"
#include "functions/MqttWrite.h"
#include <Preferences.h>
#include <cstring>
#include "helper.h"

// ================= ĐỊNH NGHĨA CÁC ĐỐI TƯỢNG CẦN CHO KẾT NỐI =================
#include "hardware/Wifi_handler.h"
#include "hardware/Sim_handler.h"

namespace {
  WiFiClient wifiMqttClient;
}

extern TinyGsm modem;
namespace {
  class MqttGsmClient : public TinyGsmClient {
   public:
    using TinyGsmClient::TinyGsmClient;
    using TinyGsmClient::write;

    size_t write(const uint8_t* data, size_t size) override {
      const size_t sent = writeMqttChunks(data, size, [this](const uint8_t* part, size_t count) {
        return TinyGsmClient::write(part, count);
      });
      if (sent != size) {
        Serial.printf("[MQTT][ERROR] Modem short write: %u/%u bytes; reconnect required\n",
                      static_cast<unsigned>(sent), static_cast<unsigned>(size));
        // The broker may have received a packet prefix. Discard this TCP stream
        // so the next CONNECT starts a fresh MQTT frame boundary.
        TinyGsmClient::stop();
      }
      return sent;
    }
  };

  MqttGsmClient gsmMqttClient(modem, 1);
  MqttGsmClient gsmProbeClient(modem, 4);
  WiFiClient wifiProbeClient;
}
PubSubClient mqtt(wifiMqttClient);

extern Preferences prefs;

namespace {
  String& mqttServerHost() {
    static String serverHost;
    return serverHost;
  }

  uint32_t lastMqttConnectAttemptMs = 0;
}

Client& activeMqttClient() {
  return isWifiConnection() ? static_cast<Client&>(wifiMqttClient)
                            : static_cast<Client&>(gsmMqttClient);
}

// ================= ĐỊNH NGHĨA HÀM =================

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  backendAgent.onMqttMessage(topic, payload, length);

  // Dispatch plain-text ATG commands separately from JSON device commands.
  if (strcmp(topic, TOPIC_SUB_CMD) == 0) {
    Serial.printf("[MQTT DOWNLINK] Nhan lenh legacy (%u bytes).\n", length);
    String cmd = "";
    for (int i = 0; i < length; i++) cmd += (char)payload[i];
    
    #if PROGRAM_DEBUG
    Serial.print("\n[MQTT DOWNLINK] Lenh: ");
    Serial.println(cmd);
    #endif

    if (cmd.isEmpty()) {
      #if PROGRAM_DEBUG
      Serial.println("[MQTT DOWNLINK] Lenh rong, khong xu ly.");
      #endif
      return;
    }
    
    // Phân tích lệnh ATG để cấu hình ESP32, kết nối hoặc receiver GNSS.
    std::vector<String> cmdWords = splitCommand(cmd);
    cmd_action_t action = handleCommand(cmdWords);
    if (action == CMD_ACTION_ESP_RESTART) {
      Serial.println("[MQTT DOWNLINK] Yeu cau ESP32 khoi dong lai.");
      shutdownTcpTransportBeforeRestart();
      ESP.restart();
    }
  }
}

bool probeMqttTransport(bool useGsm) {
  prefs.begin("myPrefs", true);
  String host = prefs.getString("MQTT_SERVER", String(MQTT_SERVER));
  if (host.equalsIgnoreCase("aitogy.asia")) host = MQTT_SERVER;
  const uint16_t port = prefs.getUShort("MQTT_PORT", MQTT_PORT);
  const String user = prefs.getString("MQTT_USER", String(MQTT_USER));
  const String password = prefs.getString("MQTT_PASS", String(MQTT_PASS));
  prefs.end();

  Client& transport = useGsm ? static_cast<Client&>(gsmProbeClient)
                             : static_cast<Client&>(wifiProbeClient);
  PubSubClient probe(transport);
  probe.setServer(host.c_str(), port);
  probe.setSocketTimeout(3);
  if (!probe.setBufferSize(1024)) return false;
  // TinyGSM connect timeout is seconds; WiFiClient uses milliseconds.
  const bool tcpReady = useGsm ? gsmProbeClient.connect(host.c_str(), port, 5)
                               : wifiProbeClient.connect(host.c_str(), port, 3000);
  // No LWT, publish or subscribe: this probe cannot change station status.
  const String clientId = "net-probe-" + backendAgent.serial();
  const bool reachable = tcpReady && probe.connect(clientId.c_str(), user.c_str(), password.c_str());
  if (reachable) probe.disconnect();
  transport.stop();
  return reachable;
}

int setupMQTT() {
  mqtt.setClient(activeMqttClient());
  if (!mqtt.setBufferSize(16384)) {
    Serial.println("[MQTT][ERROR] Cannot allocate 16384-byte MQTT buffer");
    return -1;
  }
  prefs.begin("myPrefs", false);
  mqttServerHost() = prefs.getString("MQTT_SERVER", String(MQTT_SERVER));
  if (mqttServerHost().equalsIgnoreCase("aitogy.asia")) {
    // Replace the retired aitogy.asia broker value; retain other stored hosts.
    mqttServerHost() = MQTT_SERVER;
    prefs.putString("MQTT_SERVER", mqttServerHost());
  }
  uint16_t mqttServerPort = prefs.getUShort("MQTT_PORT", MQTT_PORT);
  prefs.end();
  mqtt.setServer(mqttServerHost().c_str(), mqttServerPort);
  mqtt.setCallback(mqttCallback);
  return 0;
}

int connectMQTT() {
  if (!mqtt.connected()) {
    if (lastMqttConnectAttemptMs != 0
        && millis() - lastMqttConnectAttemptMs < 5000UL) {
      return -1;
    }
    lastMqttConnectAttemptMs = millis();
    Serial.println("\n[MQTT] Dang ket noi Broker...");
    String clientId = "agent-" + backendAgent.serial() + "-" + String(random(0xffff), HEX);
    prefs.begin("myPrefs", false);
    String mqttUser = prefs.getString("MQTT_USER", String(MQTT_USER));
    String mqttPass = prefs.getString("MQTT_PASS", String(MQTT_PASS));
    String topicSubCmd = prefs.getString("TPC_SUB_CMD", String(TOPIC_SUB_CMD));
    prefs.end();
    const String statusTopic = String("pi/devices/") + backendAgent.serial() + "/status";
    const String lwtPayload = backendAgent.lwtPayload();
    if (mqtt.connect(clientId.c_str(), mqttUser.c_str(), mqttPass.c_str(),
                    statusTopic.c_str(), 1, true, lwtPayload.c_str())) {
      Serial.println("[MQTT] Da ket noi thanh cong!");
      const String commandTopic = String("pi/devices/") + backendAgent.serial() + "/command";
      const String commandsTopic = String("pi/devices/") + backendAgent.serial() + "/commands";
      const String legacyCommandTopic = String("pi/device/") + backendAgent.serial() + "/command";
      const bool commandSubSent = mqtt.subscribe(commandTopic.c_str(), 1);
      const bool commandsSubSent = mqtt.subscribe(commandsTopic.c_str(), 1);
      const bool legacySubSent = mqtt.subscribe(legacyCommandTopic.c_str(), 1);
      const bool oldSubSent = mqtt.subscribe(topicSubCmd.c_str());
      mqtt.subscribe((String("pi/devices/") + backendAgent.serial() + "/control_ack").c_str(), 1);
      Serial.printf("[MQTT] SUBSCRIBE command=%s, commands=%s, legacy=%s, old=%s\n",
                    commandSubSent ? "OK" : "LOI", commandsSubSent ? "OK" : "LOI",
                    legacySubSent ? "OK" : "LOI", oldSubSent ? "OK" : "LOI");
      backendAgent.onMqttConnected();
      return 0;
    } else {
      Serial.print("[MQTT] Loi rc=");
      Serial.print(mqtt.state());
      Serial.println(" -> Thu lai sau 5s");
      backendAgent.onMqttDisconnected();
      return -1;
    }
  }
  return 0;
}

int publishHealth(const String& payload) {
  if (payload.isEmpty()) return -1;

  // Publish to the configured health topic under the shared transport mutex.

  if (tcpStreamMutex != nullptr && xSemaphoreTake(tcpStreamMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS))) {
    // TinyGsmClient::connected() can consume modem UART responses too.
    if (!mqtt.connected()) {
      xSemaphoreGive(tcpStreamMutex);
      return -1;
    }
    prefs.begin("myPrefs", false);
    String topicPubHealth = prefs.getString("TPC_HEALTH", String(TOPIC_PUB_HEALTH));
    prefs.end();
    bool ok = mqtt.publish(topicPubHealth.c_str(), payload.c_str());
    xSemaphoreGive(tcpStreamMutex);
    if (ok) {
      #if PROGRAM_DEBUG
      Serial.print("[MQTT] Da gui thong tin suc khoe len topic: ");
      Serial.println(topicPubHealth);
      #endif
      return 0;
    }
    #if PROGRAM_DEBUG
    Serial.println("[MQTT] publishHealth that bai, kich thuoc payload: " + String(payload.length()));
    #endif
    return -1;
  }
  #if PROGRAM_DEBUG
  Serial.println("[MQTT] publishHealth: khong lay duoc tcpStreamMutex");
  #endif
  return -1;
}

bool isMqttConnected() {
  return mqtt.connected();
}
