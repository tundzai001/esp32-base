"""Exercise the actual Wi-Fi handover against sockets that require a live route."""
from pathlib import Path
import subprocess
import tempfile
from host_coverage import compile_cpp, run_cpp, cpp_output, mapped

root = Path(__file__).resolve().parents[2]
source = (root / 'src/hardware/Network_Manager.cpp').read_text()
start = source.index('bool tryWifiFallback(bool protocolsRunning) {')
end = source.index('// Both switching directions', start)
function = source[start:end]
harness = r'''
#include <cassert>
#include <cstdint>
bool wifi = true, wifiFallbackActive = false, routeAlive = true;
bool mqttOpen = true, ntripOpen = true, probeOk = true, activated = false;
uint32_t lastSimProbeMs = 0;
uint8_t simRecoverySuccesses = 0;
bool isWifiConnection() { return wifi; }
bool isGsmConnection() { return !wifi; }
uint32_t millis() { return 123; }
struct { void println(const char *) {} } Serial;
struct { void disconnect() { assert(routeAlive); mqttOpen = false; } } mqtt;
void stopNtrip() { assert(routeAlive); ntripOpen = false; }
bool probeMqttTransport(bool gsm) { assert(!gsm); return probeOk; }
bool setupWiFi(bool (*validate)()) {
    if (wifi) assert(!mqttOpen && !ntripOpen);
    else assert(mqttOpen && ntripOpen); // Failed Wi-Fi must retain SIM traffic.
    routeAlive = false;
    return validate();
}
enum class ConnectionType { WIFI };
void activateNetwork(ConnectionType, bool) { activated = true; }
void logNetworkStatus(bool) {}
'''
checks = r'''
void reset(bool useWifi, bool success) {
    wifi = useWifi; probeOk = success;
    routeAlive = mqttOpen = ntripOpen = true;
    activated = wifiFallbackActive = false;
    lastSimProbeMs = simRecoverySuccesses = 0;
}
int main() {
    for (bool useWifi : {false, true}) {
        for (bool success : {false, true}) {
            reset(useWifi, success);
            assert(tryWifiFallback(true) == success);
            assert(activated == success);
            if (success) {
                assert(wifiFallbackActive == !useWifi);
                assert(lastSimProbeMs == 123);
            }
            if (!useWifi && !success) assert(mqttOpen && ntripOpen);
        }
    }
    reset(false, true);
    assert(tryWifiFallback(false)); // Boot has no running protocol sockets.
}
'''
function = mapped(root / 'src/hardware/Network_Manager.cpp', function)
with tempfile.TemporaryDirectory(prefix='wifi-handover-') as directory:
    path = Path(directory)
    (path / 'check.cpp').write_text('#include <initializer_list>\n' + harness + function + checks)
    compile_cpp(['c++', '-std=c++17', '-Wall', '-Wextra', '-Werror',
                    str(path / 'check.cpp'), '-o', str(path / 'check')], check=True)
    run_cpp([str(path / 'check')], check=True)
print('PASS: Wi-Fi closes sockets before route loss; SIM preserved on failed candidate; boot handover')
