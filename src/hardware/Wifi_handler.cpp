#include "Prog_Config.h"
#include "hardware/Wifi_handler.h"

bool setupWiFi()
{
  prefs.begin("myPrefs", false);
  auto ssid = prefs.getString("WIFI_SSID", "AITOGY-VP");
  auto password = prefs.getString("WIFI_PASS", "123456789");
  prefs.end();

  Serial.print("\n[WIFI] Dang ket noi mang: ");
  Serial.println(ssid);
  WiFi.begin(ssid.c_str(), password.c_str());

  auto attempt = 0;
  while (WiFiClass::status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
    if (++attempt > 10)
    {
      Serial.println("\n[ERROR] Ket noi WiFi that bai!");
      return false;
    }
  }
  Serial.println("\n[WIFI] Ket noi THANH CONG! IP: " + WiFi.localIP().toString());
  return true;
}
