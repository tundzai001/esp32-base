#define NTRIP_HANDLER_IP_CODE

#include "functions/NTRIP_Handler_IP.h"

#include "Prog_Config.h"
#include "functions/Backend_Agent.h"
#include "functions/RTCM_Receiver.h"
#include "functions/unicore_commands.h"
#include "functions/Unicore_Identification.h"
#include "hardware/Sim_handler.h"
#include "hardware/Wifi_handler.h"

#include <ArduinoJson.h>
#include <Preferences.h>
#include <cstring>

extern Preferences prefs;
extern TinyGsm modem;

namespace {
  class ntripConfig {
  public:
    inline static bool isIcyOk = false;
    inline static unsigned long lastReconnect = 0;
    inline static bool isNmeaSent = false;
  };

  // ================= HẰNG STRING =================
  inline constexpr const char nmeaCmdUnlog[] = "UNLOG\r\n";
  inline constexpr const char nmeaCmdSetBase[] = "MODE BASE TIME 120 2.5\r\n";
  inline constexpr const char rtcm1084SetOutputCom2[] = "RTCM1084 COM2 1\r\n";
  inline constexpr const char rtcm1074SetOutputCom2[] = "RTCM1074 COM2 1\r\n";
  inline constexpr const char rtcm1006SetOutputCom2[] = "RTCM1006 COM2 1\r\n";
  inline constexpr const char rtcm1124SetOutputCom2[] = "RTCM1124 COM2 1\r\n";
  inline constexpr const char rtcm1087SetOutputCom2[] = "RTCM1087 COM2 1\r\n";
  inline constexpr const char rtcm1077SetOutputCom2[] = "RTCM1077 COM2 1\r\n";
  inline constexpr const char rtcm1127SetOutputCom2[] = "RTCM1127 COM2 1\r\n";
  inline constexpr const char rtcmSetBaudCom2[] = "CONFIG COM2 115200\r\n";
  inline constexpr const char cmdSaveConfig[] = "saveconfig\r\n";
  inline constexpr uint8_t ubxMonVerPoll[] = {
    0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34
  };

  void clearGnssInput()
  {
    while (Serial1.available() > 0) {
      Serial1.read();
    }
  }

  bool hasValidUbxFrame(const uint8_t *data, size_t length)
  {
    for (size_t start = 0; start + 8 <= length; ++start) {
      if (data[start] != 0xB5 || data[start + 1] != 0x62 ||
          data[start + 2] != 0x0A || data[start + 3] != 0x04) {
        continue;
      }

      const size_t payloadLength =
          static_cast<size_t>(data[start + 4]) |
          (static_cast<size_t>(data[start + 5]) << 8);
      const size_t frameLength = payloadLength + 8;
      // A MON-VER response contains SW (30 bytes) and HW (10 bytes), never an empty poll.
      if (payloadLength < 40 || frameLength > 512) continue;
      if (start + frameLength > length) {
        continue;
      }

      uint8_t checksumA = 0;
      uint8_t checksumB = 0;
      for (size_t i = start + 2; i < start + 6 + payloadLength; ++i) {
        checksumA = static_cast<uint8_t>(checksumA + data[i]);
        checksumB = static_cast<uint8_t>(checksumB + checksumA);
      }

      if (checksumA == data[start + frameLength - 2] &&
          checksumB == data[start + frameLength - 1]) {
        return true;
      }
    }
    return false;
  }

  bool receiveUbxFrame(uint32_t timeoutMs)
  {
    uint8_t buffer[512] = {};
    size_t length = 0;
    const uint32_t start = millis();

    while (millis() - start < timeoutMs) {
      while (Serial1.available() > 0) {
        const int value = Serial1.read();
        if (length < sizeof(buffer)) {
          buffer[length++] = static_cast<uint8_t>(value);
        } else {
          memmove(buffer, buffer + 1, sizeof(buffer) - 1);
          buffer[sizeof(buffer) - 1] = static_cast<uint8_t>(value);
        }

        if (hasValidUbxFrame(buffer, length)) {
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
    while (millis() - start < timeoutMs) {
      while (Serial1.available() > 0) {
        if (identity.feed(static_cast<uint8_t>(Serial1.read()))) {
          Serial.printf("[GNSS][UNICORE RESPONSE] %s\n", identity.line);
          return true;
        }
      }
      delay(2);
    }

    return false;
  }
} // namespace

bool jsonBool(JsonObjectConst object, const String &key, bool fallback = false) {
    const JsonVariantConst value = object[key.c_str()];
    if (value.isNull()) return fallback;
    if (value.is<bool>()) return value.as<bool>();
    if (value.is<const char *>()) {
        const String text = value.as<const char *>();
        if (text.equalsIgnoreCase("true") || text == "1" || text.equalsIgnoreCase("on")) return true;
        if (text.equalsIgnoreCase("false") || text == "0" || text.equalsIgnoreCase("off")) return false;
    }
    return value.as<int>() != 0;
}

String jsonString(JsonObjectConst object, const String &key, const String &fallback = String()) {
    const JsonVariantConst value = object[key.c_str()];
    return value.isNull() ? fallback : value.as<String>();
}

bool safeNtripField(const String &value) {
    return value.indexOf('\r') < 0 && value.indexOf('\n') < 0;
}

String base64Encode(const String &plain) {
    static constexpr char alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    String encoded;
    encoded.reserve(((plain.length() + 2) / 3) * 4);
    for (size_t i = 0; i < plain.length(); i += 3) {
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

NtripEndpoint endpoints[NTRIP_SERVER_COUNT];
WiFiClient wifiNtripClient1;
WiFiClient wifiNtripClient2;
TinyGsmClient gsmNtripClient1(modem, 0);
TinyGsmClient gsmNtripClient2(modem, 2);
bool roverSessionActive = false;
bool roverSessionAuthenticated = false;
uint32_t roverLastGgaMs = 0;
uint32_t roverBytesReceived = 0;
uint32_t roverPacketsReceived = 0;

Client &clientFor(uint8_t index) {
    if (index == 0) {
        return isWifiConnection()
            ? static_cast<Client &>(wifiNtripClient1)
            : static_cast<Client &>(gsmNtripClient1);
    }
    return isWifiConnection()
        ? static_cast<Client &>(wifiNtripClient2)
        : static_cast<Client &>(gsmNtripClient2);
  }
}

void clearEndpointState(NtripEndpoint &endpoint) {
    endpoint.authenticated = false;
    endpoint.bytesInWindow = 0;
    endpoint.bytesPerSecond = 0;
    endpoint.windowStartedMs = millis();
    endpoint.lastKeepAliveMs = endpoint.windowStartedMs;
}

void loadEndpoints() {
    DynamicJsonDocument serviceDoc(CONFIG_DOCUMENT_SIZE);
    const String serialized = backendAgent.serviceConfigJson();
    const bool hasServiceConfig = deserializeJson(serviceDoc, serialized) == DeserializationError::Ok
        && serviceDoc.is<JsonObject>();
    const JsonObjectConst service = serviceDoc.as<JsonObjectConst>();

    String legacyHost;
    String legacyMountpoint;
    String legacyPassword;
    uint16_t legacyPort = NTRIP_CASTER_PORT;
    prefs.begin("myPrefs", true);
    legacyHost = prefs.getString("NTRIP_SERVER", String(NTRIP_CASTER_ADDRESS));
    legacyPort = prefs.getUShort("NTRIP_PORT", NTRIP_CASTER_PORT);
    legacyMountpoint = prefs.getString("NTRIP_MPT", String(NTRIP_MOUNTPOINT));
    legacyPassword = prefs.getString("NT_AUTH_BS", String(NTRIP_AUTH_BASE_STATION));
    prefs.end();

    for (uint8_t index = 0; index < NTRIP_SERVER_COUNT; ++index) {
        NtripEndpoint &endpoint = endpoints[index];
        const uint8_t serverId = index + 1;
        const String suffix = String(serverId);
        const String enabledKey = "server" + suffix + "_enabled";
        const String hostKey = "serverhost" + suffix;
        const String portKey = "port" + suffix;
        const String mountpointKey = "mountpoint" + suffix;
        const String usernameKey = "username" + suffix;
        const String passwordKey = "password" + suffix;
        const String versionKey = "ntrip_version" + suffix;
        const String genericHostKey = index == 0 ? String("ip") : String("ip2");
        const String genericPortKey = index == 0 ? String("port") : String("port2");
        const String genericUserKey = index == 0 ? String("user") : String("user2");
        const String genericPasswordKey = index == 0 ? String("password") : String("password2");
        const String genericMountpointKey = index == 0 ? String("mountpoint") : String("mountpoint2");

        endpoint.enabled = hasServiceConfig
            ? jsonBool(service, enabledKey, false)
            : index == 0 && !legacyHost.isEmpty();
        endpoint.host = hasServiceConfig
            ? jsonString(service, hostKey,
                         jsonString(service, "server" + suffix + "_host",
                                    jsonString(service, "host" + suffix,
                                               jsonString(service, genericHostKey,
                                                          index == 0 ? jsonString(service, "host", String()) : String()))))
            : (index == 0 ? legacyHost : String());
        endpoint.port = hasServiceConfig
            ? static_cast<uint16_t>(service[portKey.c_str()]
                | (service[("serverport" + suffix).c_str()]
                    | (service[genericPortKey.c_str()] | (index == 0 ? legacyPort : 2101))))
            : (index == 0 ? legacyPort : 2101);
        endpoint.mountpoint = hasServiceConfig
            ? jsonString(service, mountpointKey,
                         jsonString(service, "mp" + suffix,
                                    jsonString(service, genericMountpointKey, String())))
            : (index == 0 ? legacyMountpoint : String());
        endpoint.username = hasServiceConfig
            ? jsonString(service, usernameKey,
                         jsonString(service, "user" + suffix,
                                    jsonString(service, genericUserKey, "source")))
            : "source";
        endpoint.password = hasServiceConfig
            ? jsonString(service, passwordKey,
                         jsonString(service, "pass" + suffix,
                                    jsonString(service, genericPasswordKey,
                                               index == 0 ? legacyPassword : String())))
            : (index == 0 ? legacyPassword : String());
        endpoint.version = hasServiceConfig
            ? static_cast<uint8_t>(service[versionKey.c_str()] | 1)
            : 1;

        while (endpoint.mountpoint.startsWith("/")) endpoint.mountpoint.remove(0, 1);
        endpoint.enabled = endpoint.enabled && endpoint.host.length() > 0
            && endpoint.mountpoint.length() > 0 && endpoint.port > 0
            && safeNtripField(endpoint.host) && safeNtripField(endpoint.mountpoint)
            && safeNtripField(endpoint.username) && safeNtripField(endpoint.password);
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

bool waitForNtripResponse(Client &client, String *statusLine = nullptr) {
    String response;
    response.reserve(512);
    const uint32_t deadline = millis() + 10000UL;
    while ((client.connected() || client.available() > 0) && millis() < deadline && response.length() < 4096) {
        while (client.available() > 0 && response.length() < 4096) {
            const int value = client.read();
            if (value < 0) break;
            response += static_cast<char>(value);
            if (response.endsWith("\r\n\r\n")) break;
        }
        if (response.indexOf("\r\n\r\n") >= 0) break;
        delay(1);
    }
    if (statusLine) {
        const int end = response.indexOf('\n');
        *statusLine = response.substring(0, end >= 0 ? end : 240);
        statusLine->trim();
        if (statusLine->isEmpty()) *statusLine = "No response from caster (timeout or closed connection)";
    }
    return response.indexOf("ICY 200 OK") >= 0
        || response.indexOf("HTTP/1.1 200 OK") >= 0
        || response.indexOf("HTTP/1.0 200 OK") >= 0;
}

void sendRoverGgaIfAvailable(Client &client, const bool force = false) {
    if (!force && millis() - roverLastGgaMs < 5000UL) return;
    const GnssTelemetrySnapshot snapshot = getGnssTelemetrySnapshot();
    if (snapshot.lastGga[0] == '\0' || snapshot.lastGgaMs == 0
        || millis() - snapshot.lastGgaMs > 3000UL) {
        if (force || millis() - roverLastGgaMs >= 5000UL) {
            Serial.println("[NTRIP ROVER] Chua co GGA moi tu UART GNSS; chua gui GGA len caster");
            roverLastGgaMs = millis();
        }
        return;
    }
    const String outbound = String(snapshot.lastGga) + "\r\n";
    if (client.print(outbound) != outbound.length()) {
        Serial.println("[NTRIP ROVER] Loi ghi GGA len socket caster");
        roverLastGgaMs = millis();
        return;
    }
    Serial.printf("[NTRIP ROVER] GGA -> caster: %s\n", snapshot.lastGga);
    roverLastGgaMs = millis();
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
    const String auth = base64Encode(user + ":" + endpoint.password);
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

Client &activeNtripClient() {
    return clientFor(0);
}

int bootstrapUM980() {
  // Gửi lệnh ASCII cấu hình output và survey-in cho Unicore trên COM2.
  // Nhánh khởi tạo này chỉ gửi UART và ghi log, chưa kiểm tra ACK của từng lệnh.
  unsigned long WaitStartTime = millis();
  while (!Serial1.available() && millis() - WaitStartTime < 2000) {
    delay(10);
  }

  Serial1.write(nmeaCmdUnlog, strlen(nmeaCmdUnlog)); // Gửi lệnh UNLOG

  #ifdef PROGRAM_DEBUG
  WaitStartTime = millis();
  while (!Serial1.available() && millis() - WaitStartTime < 2000) {
    delay(10);
  }
  String nmeaResponse = Serial1.readStringUntil('\n'); // Đọc phản hồi từ GNSS
  Serial.println("[NMEA CMD] Response: " + nmeaResponse);
  #endif
  delay(20);

  // cài baud rate cho COM2
  WaitStartTime = millis();
  while (!Serial1.available() && millis() - WaitStartTime < 2000) {
    delay(10);
  }
  Serial1.write(rtcmSetBaudCom2, strlen(rtcmSetBaudCom2)); // Đặt baud COM2 bằng lệnh ASCII.

  // thiết lập chế độ base
  WaitStartTime = millis();
  while (!Serial1.available() && millis() - WaitStartTime < 2000) {
    delay(10);
  }
  Serial1.write(nmeaCmdSetBase, strlen(nmeaCmdSetBase)); // Bắt đầu survey-in bằng lệnh ASCII.

  #ifdef PROGRAM_DEBUG
  WaitStartTime = millis();
  while (!Serial1.available() && millis() - WaitStartTime < 2000) {
    delay(10);
  }
  nmeaResponse = Serial1.readStringUntil('\n'); // Đọc phản hồi từ GNSS
  Serial.println("[NMEA CMD] Response: " + nmeaResponse);
  #endif
  delay(20);

  // gửi rtcm 1084 ra COM2
  WaitStartTime = millis();
  while (!Serial1.available() && millis() - WaitStartTime < 2000) {
    delay(10);
  }
  Serial1.write(rtcm1084SetOutputCom2, strlen(rtcm1084SetOutputCom2)); // Gửi lệnh RTCM để thiết lập đầu ra COM2

  #ifdef PROGRAM_DEBUG
  WaitStartTime = millis();
  while (!Serial1.available() && millis() - WaitStartTime < 2000) {
    delay(10);
  }
  nmeaResponse = Serial1.readStringUntil('\n'); // Đọc phản hồi từ GNSS
  Serial.println("[NMEA CMD] Response: " + nmeaResponse);
  #endif
  delay(20);

  // gửi rtcm 1074 ra COM2
  WaitStartTime = millis();
  while (!Serial1.available() && millis() - WaitStartTime < 2000) {
    delay(10);
  }
  Serial1.write(rtcm1074SetOutputCom2, strlen(rtcm1074SetOutputCom2)); // Gửi lệnh RTCM để thiết lập đầu ra COM2

  #ifdef PROGRAM_DEBUG
  WaitStartTime = millis();
  while (!Serial1.available() && millis() - WaitStartTime < 2000) {
    delay(10);
  }
  nmeaResponse = Serial1.readStringUntil('\n'); // Đọc phản hồi từ GNSS
  Serial.println("[NMEA CMD] Response: " + nmeaResponse);
  #endif
  delay(20);

  // gửi rtcm 1006 ra COM2
  WaitStartTime = millis();
  while (!Serial1.available() && millis() - WaitStartTime < 2000) {
    delay(10);
  }
  Serial1.write(rtcm1006SetOutputCom2, strlen(rtcm1006SetOutputCom2)); // Gửi lệnh RTCM để thiết lập đầu ra COM2

  #ifdef PROGRAM_DEBUG
  WaitStartTime = millis();
  while (!Serial1.available() && millis() - WaitStartTime < 2000) {
    delay(10);
  }
  nmeaResponse = Serial1.readStringUntil('\n'); // Đọc phản hồi từ GNSS
  Serial.println("[NMEA CMD] Response: " + nmeaResponse);
  #endif
  delay(20);

  // gửi rtcm 1124 ra COM2
  WaitStartTime = millis();
  while (!Serial1.available() && millis() - WaitStartTime < 2000) {
    delay(10);
  }
  Serial1.write(rtcm1124SetOutputCom2, strlen(rtcm1124SetOutputCom2)); // Gửi lệnh RTCM để thiết lập đầu ra COM2

  #ifdef PROGRAM_DEBUG
  WaitStartTime = millis();
  while (!Serial1.available() && millis() - WaitStartTime < 2000) {
    delay(10);
  }
  nmeaResponse = Serial1.readStringUntil('\n'); // Đọc phản hồi từ GNSS
  Serial.println("[NMEA CMD] Response: " + nmeaResponse);
  #endif
  delay(20);

  // gửi rtcm 1087 ra COM2
  WaitStartTime = millis();
  while (!Serial1.available() && millis() - WaitStartTime < 2000) {
    delay(10);
  }
  Serial1.write(rtcm1087SetOutputCom2, strlen(rtcm1087SetOutputCom2)); // Gửi lệnh RTCM để thiết lập đầu ra COM2

  #ifdef PROGRAM_DEBUG
  WaitStartTime = millis();
  while (!Serial1.available() && millis() - WaitStartTime < 2000) {
    delay(10);
  }
  nmeaResponse = Serial1.readStringUntil('\n'); // Đọc phản hồi từ GNSS
  Serial.println("[NMEA CMD] Response: " + nmeaResponse);
  #endif
  delay(20);

  // gửi rtcm 1077 ra COM2
  WaitStartTime = millis();
  while (!Serial1.available() && millis() - WaitStartTime < 2000) {
    delay(10);
  }
  Serial1.write(rtcm1077SetOutputCom2, strlen(rtcm1077SetOutputCom2)); // Gửi lệnh RTCM để thiết lập đầu ra COM2

  #ifdef PROGRAM_DEBUG
  WaitStartTime = millis();
  while (!Serial1.available() && millis() - WaitStartTime < 2000) {
    delay(10);
  }
  nmeaResponse = Serial1.readStringUntil('\n'); // Đọc phản hồi từ GNSS
  Serial.println("[NMEA CMD] Response: " + nmeaResponse);
  #endif
  delay(20);

  // gửi rtcm 1127 ra COM2
  WaitStartTime = millis();
  while (!Serial1.available() && millis() - WaitStartTime < 2000) {
    delay(10);
  }
  Serial1.write(rtcm1127SetOutputCom2, strlen(rtcm1127SetOutputCom2)); // Gửi lệnh RTCM để thiết lập đầu ra COM2

  #ifdef PROGRAM_DEBUG
  WaitStartTime = millis();
  while (!Serial1.available() && millis() - WaitStartTime < 2000) {
    delay(10);
  }
  nmeaResponse = Serial1.readStringUntil('\n'); // Đọc phản hồi từ GNSS
  Serial.println("[NMEA CMD] Response: " + nmeaResponse);
  #endif
  delay(20);

  #ifdef PROGRAM_DEBUG
  WaitStartTime = millis();
  while (!Serial1.available() && millis() - WaitStartTime < 2000) {
    delay(10);
  }
  nmeaResponse = Serial1.readStringUntil('\n'); // Đọc phản hồi từ GNSS
  Serial.println("[NMEA CMD] Response: " + nmeaResponse);
  #endif
  delay(20);

  // Send ephemeris output commands before saving the receiver configuration.
  for (const auto &command : UnicoreCommands::buildEphemerisOutputCommands("COM2")) {
    if (Serial1.write(command.data(), command.size()) != command.size()) {
      Serial.println("[UNICORE][ERROR] Khong gui duoc lenh bat ephemeris.");
      return -1;
    }
    Serial1.flush();
    delay(100);
  }

  // lưu cấu hình
  WaitStartTime = millis();
  while (!Serial1.available() && millis() - WaitStartTime < 2000) {
    delay(10);
  }
  Serial1.write(cmdSaveConfig, strlen(cmdSaveConfig)); // Gửi lệnh lưu cấu hình

  #ifdef PROGRAM_DEBUG
  WaitStartTime = millis();
  while (!Serial1.available() && millis() - WaitStartTime < 2000) {
    delay(10);
  }
  nmeaResponse = Serial1.readStringUntil('\n'); // Đọc phản hồi từ GNSS
  Serial.println("[NMEA CMD] Response: " + nmeaResponse);
  #endif
  delay(20);
  return 0;
}

int bootstrapUblox() {
  GnssCmdBuilder::GnssOptions options;
  options.ports = {"UART1"};
  options.outputMode = GnssCmdBuilder::OutputMode::DiagnosticsRtcm;
  options.msmLevel = GnssCmdBuilder::MsmLevel::Both;

  const GnssCmdBuilder::Command outputConfig =
      GnssCmdBuilder::buildUbloxOutputConfigCommand(options);
  if (outputConfig.empty()) {
    Serial.println("[UBLOX][ERROR] Khong tao duoc lenh cau hinh output.");
    return -1;
  }

  Serial1.write(outputConfig.data(), outputConfig.size());
  Serial1.flush();
  delay(100);

  const GnssCmdBuilder::Command saveConfig = {
      0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00,
      0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x03, 0x1D, 0xAB};
  Serial1.write(saveConfig.data(), saveConfig.size());
  Serial1.flush();

  Serial.println("[UBLOX] Da cau hinh output RTCM/NMEA tren UART1.");
  return 0;
}

GnssCmdBuilder::GnssSensorType detectGnssSensorType() {
  Serial.println("[GNSS] Cho receiver khoi dong 2000 ms truoc khi probe.");
  delay(2000);
  const uint32_t baudRates[] = {GNSS_BAUD, 38400, 9600, 19200, 57600, 230400, 460800};
  for (const uint32_t baud : baudRates) {
    Serial1.updateBaudRate(baud);
    delay(30);
    Serial.printf("[GNSS] Probe baud=%lu\n", static_cast<unsigned long>(baud));
    clearGnssInput();
    Serial1.write(ubxMonVerPoll, sizeof(ubxMonVerPoll));
    Serial1.flush();
    if (receiveUbxFrame(1000)) {
      Serial.printf("[GNSS] Da nhan dien u-blox qua UBX-MON-VER, baud=%lu.\n", static_cast<unsigned long>(baud));
      return GnssCmdBuilder::GnssSensorType::Ublox;
    }

    clearGnssInput();
    Serial1.print("VERSIONA\r\n");
    Serial1.flush();
    delay(50);
    if (receiveUnicoreIdentification(1000)) {
      Serial.printf("[GNSS] Da nhan dien Unicore qua lenh VERSIONA, baud=%lu.\n", static_cast<unsigned long>(baud));
      return GnssCmdBuilder::GnssSensorType::Unicore;
    }
  }

  Serial1.updateBaudRate(GNSS_BAUD);
  delay(30);
  clearGnssInput();
  Serial.printf("[GNSS] Thu lai VERSIONA tai baud=%lu sau khi receiver da khoi dong.\n",
                static_cast<unsigned long>(GNSS_BAUD));
  Serial1.print("VERSIONA\r\n");
  Serial1.flush();
  if (receiveUnicoreIdentification(1500)) return GnssCmdBuilder::GnssSensorType::Unicore;
  Serial.println("[GNSS][ERROR] Khong the nhan dien chip GNSS.");
  return GnssCmdBuilder::GnssSensorType::Invalid;
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

int loopNTRIP(String &rtcmData) {
    if (roverSessionActive) return 504;

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
            if (millis() - endpoint.lastReconnectMs >= RECONNECT_INTERVAL_MS) {
                connectEndpoint(index);
            }
            continue;
        }

        anyConnected = true;
        if (!rtcmData.isEmpty()) {
            const size_t payloadLength = rtcmData.length();
            size_t sent = 0;
            if (endpoint.version == 2) {
                // NTRIP v2 source streams use HTTP chunk framing after the
                // POST handshake. The payload itself remains untouched.
                client.print(String(payloadLength, HEX) + "\r\n");
                sent = client.write(reinterpret_cast<const uint8_t *>(rtcmData.c_str()), payloadLength);
                client.print("\r\n");
            } else {
                sent = client.write(reinterpret_cast<const uint8_t *>(rtcmData.c_str()), payloadLength);
            }
            if (sent == payloadLength) {
                endpoint.bytesInWindow += static_cast<uint32_t>(sent);
                anySent = true;
            }
        } else if (endpoint.version == 2
                   && millis() - endpoint.lastKeepAliveMs >= 10000UL) {
            client.print("0\r\n\r\n");
            endpoint.lastKeepAliveMs = millis();
        }
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

bool startNtripRover(const String &host, const uint16_t port, const String &username,
                    const String &password, String mountpoint, const uint8_t version) {
    if (host.isEmpty() || port == 0 || mountpoint.isEmpty()
        || !safeNtripField(host) || !safeNtripField(username)
        || !safeNtripField(password) || !safeNtripField(mountpoint)) return false;
    while (mountpoint.startsWith("/")) mountpoint.remove(0, 1);
    if (mountpoint.isEmpty()) return false;

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
    if (!client.connect(host.c_str(), port)) {
        roverSessionActive = false;
        return false;
    }

    const String user = username.isEmpty() ? String("source") : username;
    const String auth = base64Encode(user + ":" + password);
    String request;
    if (version == 2) {
        request = "GET /" + mountpoint + " HTTP/1.1\r\n"
                  "Host: " + host + ":" + String(port) + "\r\n"
                  "Ntrip-Version: Ntrip/2.0\r\n"
                  "User-Agent: NTRIP ESP32-Agent/" + String(AGENT_VERSION) + "\r\n"
                  "Authorization: Basic " + auth + "\r\n"
                  "Accept: */*\r\n"
                  "Connection: Keep-Alive\r\n\r\n";
    } else {
        request = "GET /" + mountpoint + " HTTP/1.0\r\n"
                  "User-Agent: NTRIP ESP32-Agent/" + String(AGENT_VERSION) + "\r\n"
                  "Authorization: Basic " + auth + "\r\n"
                  "Accept: */*\r\n"
                  "Connection: close\r\n\r\n";
    }
    client.print(request);

    if (!waitForNtripResponse(client)) {
        client.stop();
        roverSessionActive = false;
        return false;
    }
    roverSessionAuthenticated = true;
    sendRoverGgaIfAvailable(client, true);
    Serial.println("[NTRIP ROVER] Da ket noi CORS " + host + ":" + String(port)
                   + "/" + mountpoint);
    return true;
}

int loopNtripRover() {
    if (!roverSessionActive || !roverSessionAuthenticated) return 504;
    Client &client = clientFor(0);
    if (!client.connected()) {
        roverSessionActive = false;
        roverSessionAuthenticated = false;
        return 504;
    }
    sendRoverGgaIfAvailable(client);
    if (client.available() <= 0) return 200;

    uint8_t buffer[512] = {};
    if (!rtcmBufferMutex
        || xSemaphoreTake(rtcmBufferMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) != pdTRUE) {
        return 200;
    }
    const int available = client.available();
    const size_t bytesToRead = min(static_cast<size_t>(available), sizeof(buffer));
    const int bytesRead = client.read(buffer, bytesToRead);
    if (bytesRead > 0) {
        Serial1.write(buffer, static_cast<size_t>(bytesRead));
        roverBytesReceived += static_cast<uint32_t>(bytesRead);
        ++roverPacketsReceived;
    }
    xSemaphoreGive(rtcmBufferMutex);
    return bytesRead > 0 ? bytesRead : 200;
}

void stopNtripRover() {
    if (roverSessionActive) clientFor(0).stop();
    roverSessionActive = false;
    roverSessionAuthenticated = false;
    roverLastGgaMs = 0;
}

bool ntripRoverActive() {
    return roverSessionActive && roverSessionAuthenticated;
}

uint32_t ntripRoverBytesReceived() { return roverBytesReceived; }
uint32_t ntripRoverPacketsReceived() { return roverPacketsReceived; }
