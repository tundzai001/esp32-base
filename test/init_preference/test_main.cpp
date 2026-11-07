#include "Top_Lvl_Config.h"
#include <unity.h>
#include <Arduino.h>
#include "Prog_Config.h"
#include "Preferences.h"

Preferences prefs;

void setUp(void)
{
}

void tearDown(void)
{
}

void setup()
{
  prefs.begin("myPrefs");
  prefs.putBool("NOT_FIRST_BOOT", true);
  prefs.putString("APN", "v-internet");
  prefs.putString("GPRS_USER", "");
  prefs.putString("GPRS_PASS", "");
  prefs.putString("NTRIP_SERVER", "ntrip.aitogy.com");
  prefs.putInt("NTRIP_PORT", 2101);
  prefs.putString("NTRIP_MOUNTPOINT", "/test");
  prefs.putString("NTRIP_AUTH_BASE_STATION", "12345");
  prefs.putString("MQTT_SERVER", "aitogy.asia");
  prefs.putInt("MQTT_PORT", 1883);
  prefs.putString("MQTT_USER", "mqttUser");
  prefs.putString("MQTT_PASS", "MqttPassword123$%^");
  prefs.putString("TOPIC_SUB_CMD", "tdm2402/um980_base_001/cmd");
  prefs.putString("TOPIC_PUB_HEALTH", "tdm2402/um980_base_001/health");
}

void loop()
{
  // Preferences are written once during setup; no periodic work is required.
}
