"""Run real Wi-Fi setup against absent legacy NVS and saved/missing credentials."""
from pathlib import Path
import subprocess
import tempfile
from host_coverage import compile_cpp, run_cpp, cpp_output, mapped

root = Path(__file__).resolve().parents[2]
source = (root / 'src/hardware/Wifi_handler.cpp').read_text()
setup = source[source.index('namespace {'):]
shim = (root / 'test/host/Arduino.h').read_text().replace(
    '    void println(const char *) {}',
    '    void println(const char *) {}\n    void println(const std::string &) {}')
harness = r'''
#include <Arduino.h>
#include <ArduinoJson.h>
#include <cassert>
#include <cstring>
#include <functional>
#include <memory>
constexpr int WIFI_STA = 1, WL_CONNECTED = 3, ESP_OK = 0, NVS_READONLY = 1;
constexpr const char *WIFI_SSID = "fallback", *WIFI_PASSWORD = "fallback-password";
using nvs_handle_t = int;
bool credentialsExist = true, routeOk = true;
unsigned saves = 0, legacyChecks = 0;
String triedSsid, triedPassword;
uint32_t clockMs = 0;
uint32_t millis() { return clockMs; }
void delay(unsigned ms) { clockMs += ms; }
struct IPAddress { String toString() const { return "192.0.2.1"; } };
struct WiFiClass {
    inline static int connectionStatus = 0;
    static void mode(int) {}
    static int status() { return connectionStatus; }
    void persistent(bool) {}
    void disconnect(bool, bool) { connectionStatus = 0; }
    void begin(const char *ssid, const char *password) {
        triedSsid = ssid; triedPassword = password;
        connectionStatus = routeOk ? WL_CONNECTED : 0;
    }
    IPAddress localIP() const { return {}; }
} WiFi;
struct Preferences {
    bool begin(const char *space, bool readOnly) { assert(!strcmp(space, "myPrefs") && readOnly); return true; }
    bool isKey(const char *) { return credentialsExist; }
    String getString(const char *key, const char *fallback) {
        return credentialsExist ? (!strcmp(key, "WIFI_SSID") ? "saved-network" : "saved-password") : fallback;
    }
    void end() {}
};
int nvs_open(const char *space, int mode, nvs_handle_t *) {
    assert(!strcmp(space, "wifi_config") && mode == NVS_READONLY); ++legacyChecks;
    return -1; // Optional old namespace is absent; Preferences must never open it.
}
int nvs_get_str(nvs_handle_t, const char *, char *, size_t *) { return -1; }
void nvs_close(nvs_handle_t) {}
bool validWifiSsid(const String &ssid) { return !ssid.empty(); }
bool validWifiPassword(const String &) { return true; }
bool saveMigratedWifi(const String &ssid, const String &password) {
    assert(ssid == triedSsid && password == triedPassword);
    ++saves; credentialsExist = true; return true;
}
'''
checks = r'''
int main() {
    assert(setupWiFi(nullptr));
    assert(triedSsid == "saved-network" && triedPassword == "saved-password" && saves == 0);
    credentialsExist = false;
    assert(setupWiFi(nullptr));
    assert(triedSsid == WIFI_SSID && triedPassword == WIFI_PASSWORD && saves == 1);
    assert(setupWiFi(nullptr));
    assert(triedSsid == "saved-network" && saves == 1); // Next boot reads saved values.
    credentialsExist = false; routeOk = false;
    assert(!setupWiFi(nullptr) && saves == 1 && legacyChecks == 4);
}
'''
setup = mapped(root / 'src/hardware/Wifi_handler.cpp', setup)
with tempfile.TemporaryDirectory(prefix='esp-wifi-nvs-') as directory:
    path = Path(directory)
    (path / 'Arduino.h').write_text(shim)
    (path / 'check.cpp').write_text(harness + setup + checks)
    compile_cpp(['c++', '-std=c++17', '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined',
                    '-I' + str(path), '-I' + str(root / 'test/host'), '-I' + str(root / '.pio/libdeps/tdm240x_dev_general/ArduinoJson/src'),
                    str(path / 'check.cpp'), '-o', str(path / 'check')], check=True)
    run_cpp([str(path / 'check')], check=True)
print('PASS: saved Wi-Fi reused; missing keys saved only after successful connection; absent legacy NVS is optional')
