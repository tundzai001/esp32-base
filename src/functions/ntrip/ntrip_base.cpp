#define NTRIP_HANDLER_IP_CODE

#include "functions/ntrip/ntrip_base.h"
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

bool connectEndpoint(uint8_t index)
{
  if (index >= NTRIP_SERVER_COUNT || !endpoints[index].enabled || !streamActive(index))
    return false;
  NtripEndpoint &endpoint = endpoints[index];
  Client &client = clientFor(index);
  endpoint.lastReconnectMs = millis();
  client.stop();
  client.setTimeout(3000);

  Serial.println("[NTRIP] Ket noi S" + String(index + 1) + ": " + endpoint.host + ":" + String(endpoint.port) + "/" + endpoint.mountpoint);
  if (!client.connect(endpoint.host.c_str(), endpoint.port))
  {
    endpoint.lastError = "TCP connection failed";
    Serial.printf("[NTRIP][ERROR] S%u: %s\n", index + 1, endpoint.lastError.c_str());
    clearEndpointState(endpoint);
    return false;
  }

  const auto user = endpoint.username.isEmpty() ? String("source") : endpoint.username;
  const auto auth = base64Encode(user + ":" + endpoint.password);
  String request;
  if (endpoint.version == 2)
  {
    request = "POST /" + endpoint.mountpoint + " HTTP/1.1\r\n"
                                               "Host: " +
              endpoint.host + ":" + String(endpoint.port) + "\r\n"
                                                            "Ntrip-Version: Ntrip/2.0\r\n"
                                                            "User-Agent: NTRIP ESP32-Agent/" +
              String(AGENT_VERSION) + "\r\n"
                                      "Authorization: Basic " +
              auth + "\r\n"
                     "Transfer-Encoding: chunked\r\n"
                     "Connection: Keep-Alive\r\n\r\n";
  }
  else
  {
    request = "SOURCE " + endpoint.password + " /" + endpoint.mountpoint + " HTTP/1.1\r\n"
                                                                           "Host: " +
              endpoint.host + ":" + String(endpoint.port) + "\r\n"
                                                            "Ntrip-Version: Ntrip/2.0\r\n"
                                                            "User-Agent: NTRIP ESP32-Agent/" +
              String(AGENT_VERSION) + "\r\n"
                                      "Authorization: Basic " +
              auth + "\r\n"
                     "Connection: Keep-Alive\r\n\r\n";
  }
  client.print(request);

  String responseLine;
  if (!waitForNtripResponse(client, &responseLine))
  {
    if (!endpoint.password.isEmpty())
      responseLine.replace(endpoint.password, "[redacted]");
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

void ntrip_common::updateBps(NtripEndpoint &endpoint)
{
  const uint32_t now = millis();
  if (endpoint.windowStartedMs == 0)
    endpoint.windowStartedMs = now;
  if (now - endpoint.windowStartedMs >= BPS_WINDOW_MS)
  {
    const uint32_t elapsed = now - endpoint.windowStartedMs;
    endpoint.bytesPerSecond = elapsed > 0
                                  ? static_cast<uint32_t>((static_cast<uint64_t>(endpoint.bytesInWindow) * 1000ULL) / elapsed)
                                  : endpoint.bytesInWindow;
    endpoint.bytesInWindow = 0;
    endpoint.windowStartedMs = now;
  }
}

Client &activeNtripClient()
{
  return clientFor(0);
}

bool ntrip_common::sendEndpointRtcm(NtripEndpoint &endpoint, Client &client, const String &rtcmData)
{
  if (rtcmData.isEmpty())
  {
    if (endpoint.version == 2 && millis() - endpoint.lastKeepAliveMs >= 10000UL)
    {
      client.print("0\r\n\r\n");
      endpoint.lastKeepAliveMs = millis();
    }
    return false;
  }

  const auto payloadLength = rtcmData.length();
  auto sent = 0;
  if (endpoint.version == 2)
  {
    client.print(String(payloadLength, HEX) + "\r\n");
    sent = client.write(reinterpret_cast<const uint8_t *>(rtcmData.c_str()), payloadLength);
    client.print("\r\n");
  }
  else
  {
    sent = client.write(reinterpret_cast<const uint8_t *>(rtcmData.c_str()), payloadLength);
  }
  if (sent != payloadLength)
    return false;
  endpoint.bytesInWindow += static_cast<uint32_t>(sent);
  return true;
}

int bootstrapUM980()
{
  sendUm980Command(nmeaCmdUnlog, 8, true, 20);
  sendUm980Command(rtcmSetBaudCom2, 21, false, 0);
  sendUm980Command(nmeaCmdSetBase, 25, true, 20);
  sendUm980Command(rtcm1084SetOutputCom2, 18, true, 20);
  sendUm980Command(rtcm1074SetOutputCom2, 18, true, 20);
  sendUm980Command(rtcm1006SetOutputCom2, 18, true, 20);
  sendUm980Command(rtcm1124SetOutputCom2, 18, true, 20);
  sendUm980Command(rtcm1087SetOutputCom2, 18, true, 20);
  sendUm980Command(rtcm1077SetOutputCom2, 18, true, 20);
  sendUm980Command(rtcm1127SetOutputCom2, 18, true, 20);
  logUm980Response();
  delay(20);

  // Send ephemeris output commands before saving the receiver configuration.
  for (const auto &command : UnicoreCommands::buildEphemerisOutputCommands("COM2"))
  {
    if (Serial1.write(command.data(), command.size()) != command.size())
    {
      Serial.println("[UNICORE][ERROR] Khong gui duoc lenh bat ephemeris.");
      return -1;
    }
    Serial1.flush();
    delay(100);
  }

  sendUm980Command(cmdSaveConfig, 13, true, 20);
  return 0;
}

int bootstrapUblox()
{
  GnssCmdBuilder::GnssOptions options;
  options.ports = {"UART1"};
  options.outputMode = GnssCmdBuilder::OutputMode::DiagnosticsRtcm;
  options.msmLevel = GnssCmdBuilder::MsmLevel::Both;

  const GnssCmdBuilder::Command outputConfig =
      GnssCmdBuilder::buildUbloxOutputConfigCommand(options);
  if (outputConfig.empty())
  {
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

GnssCmdBuilder::GnssSensorType detectGnssSensorType()
{
  Serial.println("[GNSS] Cho receiver khoi dong 2000 ms truoc khi probe.");
  delay(2000);
  const std::array<uint32_t, 7> baudRates = {
      GNSS_BAUD, 38400, 9600, 19200, 57600, 230400, 460800};
  for (const uint32_t baud : baudRates)
  {
    Serial1.updateBaudRate(baud);
    delay(30);
    Serial.printf("[GNSS] Probe baud=%lu\n", static_cast<unsigned long>(baud));
    clearGnssInput();
    Serial1.write(ubxMonVerPoll.data(), ubxMonVerPoll.size());
    Serial1.flush();
    if (receiveUbxFrame(1000))
    {
      Serial.printf("[GNSS] Da nhan dien u-blox qua UBX-MON-VER, baud=%lu.\n", static_cast<unsigned long>(baud));
      return GnssCmdBuilder::GnssSensorType::Ublox;
    }

    clearGnssInput();
    Serial1.print("VERSIONA\r\n");
    Serial1.flush();
    delay(50);
    if (receiveUnicoreIdentification(1000))
    {
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
  if (receiveUnicoreIdentification(1500))
    return GnssCmdBuilder::GnssSensorType::Unicore;
  Serial.println("[GNSS][ERROR] Khong the nhan dien chip GNSS.");
  return GnssCmdBuilder::GnssSensorType::Invalid;
}

int setupNTRIP()
{
  for (uint8_t index = 0; index < NTRIP_SERVER_COUNT; ++index)
    stopEndpoint(index);
  loadEndpoints();
  return 0;
}

int connectNTRIP()
{
  loadEndpoints();
  auto connected = 0;
  for (uint8_t index = 0; index < NTRIP_SERVER_COUNT; ++index)
  {
    if (connectEndpoint(index))
      ++connected;
  }
  return connected > 0 ? 0 : -1;
}

int loopNTRIP(String &rtcmData)
{
  if (roverSessionActive)
    return 504;

  auto returnCode = NTRIP_MODE;
  auto anyConnected = false;
  auto anySent = false;

  for (uint8_t index = 0; index < NTRIP_SERVER_COUNT; ++index)
  {
    NtripEndpoint &endpoint = endpoints[index];
    Client &client = clientFor(index);
    if (!endpoint.enabled || !streamActive(index))
    {
      if (client.connected())
        client.stop();
      clearEndpointState(endpoint);
      continue;
    }

    if (!client.connected() || !endpoint.authenticated)
    {
      endpoint.authenticated = false;
      if (millis() - endpoint.lastReconnectMs >= RECONNECT_INTERVAL_MS)
      {
        connectEndpoint(index);
      }
      continue;
    }

    anyConnected = true;
    anySent = sendEndpointRtcm(endpoint, client, rtcmData) || anySent;
    updateBps(endpoint);
  }

  if (anySent)
    returnCode += 4;
  if (!anyConnected)
    returnCode = 504;
  return returnCode;
}

void stopNtrip()
{
  for (uint8_t index = 0; index < NTRIP_SERVER_COUNT; ++index)
    stopEndpoint(index);
  roverSessionActive = false;
  roverSessionAuthenticated = false;
  roverLastGgaMs = 0;
}

bool isNtripConnected()
{
  return ntripServerConnected(1) || ntripServerConnected(2);
}

bool ntripServerConnected(uint8_t serverId)
{
  if (serverId < 1 || serverId > NTRIP_SERVER_COUNT)
    return false;
  const uint8_t index = serverId - 1;
  return endpoints[index].authenticated && clientFor(index).connected();
}

uint32_t ntripServerBps(uint8_t serverId)
{
  if (serverId < 1 || serverId > NTRIP_SERVER_COUNT)
    return 0;
  updateBps(endpoints[serverId - 1]);
  return endpoints[serverId - 1].bytesPerSecond;
}

String ntripServerError(uint8_t serverId)
{
  if (serverId < 1 || serverId > NTRIP_SERVER_COUNT)
    return String();
  return endpoints[serverId - 1].lastError;
}
