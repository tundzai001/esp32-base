"""Exercise the actual status builder and stream commands without network hardware.

Pass a previous firmware root to compare all JSON outputs across a refactor.
"""
from pathlib import Path
import json
import re
import subprocess
import sys
import tempfile
from host_coverage import compile_cpp, run_cpp, cpp_output, mapped

root = Path(__file__).resolve().parents[2]
shim = (root / 'test/host/Arduino.h').read_text().replace('#pragma once', '')
shim = shim.replace('    void trim() {', '''    String(const char *text) : std::string(text ? text : "") {}
    String &operator=(const char *text) { assign(text ? text : ""); return *this; }
    long toInt() const { return std::strtol(c_str(), nullptr, 10); }
    bool startsWith(const String &prefix) const { return compare(0, prefix.size(), prefix) == 0; }
    int indexOf(const String &value) const { auto p = find(value); return p == npos ? -1 : int(p); }
    void remove(size_t start, size_t count) { erase(start, count); }
    void toLowerCase() { std::transform(begin(), end(), begin(), [](unsigned char c) { return std::tolower(c); }); }
    bool equalsIgnoreCase(String value) const { String own = *this; own.toLowerCase(); value.toLowerCase(); return own == value; }
    void trim() {''')
snapshot = (root / 'include/functions/ntrip/RTCM_Receiver.h').read_text().split('struct GnssTelemetrySnapshot {', 1)[1].split('\n};', 1)[0]
common = r'''
#include <cassert>
#include <array>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <vector>
#include <ArduinoJson.h>
#include "functions/backend/Station_Lock.h"
#include "functions/gnss/Ubx_Navigation.h"
class Stream;
using QueueHandle_t = void *;
using SemaphoreHandle_t = void *;
using TaskHandle_t = void *;
namespace GnssCmdBuilder { enum class GnssSensorType { Invalid, Ublox, Unicore }; using CommandList = std::vector<std::vector<uint8_t>>; }
constexpr size_t CONFIG_DOCUMENT_SIZE = 4096, STATUS_DOCUMENT_SIZE = 8192, VERIFICATION_DOCUMENT_SIZE = 12288;
constexpr unsigned BACKEND_CONTROL_STALE_MS = 30000, BACKEND_CONTROL_RECOVERY_MS = 10000;
constexpr const char *AGENT_VERSION = "test", *MQTT_SERVER = "broker", *BACKEND_HOST = "backend";
unsigned now = 10000, saves = 0;
uint32_t millis() { return now; }
struct { unsigned baudRate() { return 115200; } } Serial1;
struct { int RSSI() { return -50; } } WiFi;
struct { int getSignalQuality() { return 20; } } modem;
struct {
    unsigned getHeapSize() { return 300000; }
    unsigned getFreeHeap() { return 150000; }
    unsigned getCpuFreqMHz() { return 240; }
    unsigned getFlashChipSize() { return 4194304; }
} ESP;
bool isWifiConnection() { return true; }
bool isGsmConnection() { return false; }
unsigned getRtcmInputBps() { return 100; }
unsigned ntripServerBps(unsigned id) { return id * 50; }
bool ntripServerConnected(unsigned id) { return id == 1; }
String ntripServerError(unsigned) { return ""; }
unsigned ntripRoverBytesReceived() { return 500; }
unsigned ntripRoverPacketsReceived() { return 10; }
void appendStorageStatus(JsonObject system) { system["nvs"]["cleanup_supported"] = true; }
namespace BackendAgentInternal {
String epochString() { return "1700000000"; }
String tokenFingerprint(const String &) { return "fingerprint"; }
String readPreference(const char *, const String &fallback) { return fallback; }
bool jsonBool(JsonObjectConst object, const String &key, bool fallback = false) {
    return object[key].isNull() ? fallback : object[key].as<bool>();
}
String jsonString(JsonObjectConst object, const String &key, const String &fallback = String()) {
    return object[key].isNull() ? fallback : object[key].as<String>();
}
void copyJsonObject(JsonDocument &doc, const char *key, const String &text) {
    DynamicJsonDocument source(4096);
    deserializeJson(source, text); doc[key].set(source.as<JsonObjectConst>());
}
}
'''
checks = r'''
bool BackendAgent::mqttConnected() const { return true; }
bool BackendAgent::websocketConnected() const { return false; }
bool BackendAgent::saveStatePreferences() { ++saves; return true; }
int main() {
    for (const char *state : {"online", "configuring", "rebooting", "rebooting_for_reset", "auto_setup_base", "reference_check_queued"})
        for (unsigned flags = 0; flags < 8; ++flags)
            for (const char *chip : {"UNKNOWN", "Ublox", "Unicorecomm"}) {
                BackendAgent agent;
                agent.serialNumber_ = "test"; agent.agentState_ = state;
                agent.licenseValid_ = flags & 1; agent.provisioned_ = flags & 2;
                agent.stationLock_.all = flags & 4; agent.sensorType_ = chip;
                agent.serviceConfig_ = R"({"stream_on_demand":true,"server1_enabled":true,"server2_enabled":true,"server1_stream_active":true})";
                agent.navigationMs_ = agent.dopMs_ = agent.unicoreQualityMs_ = 9000;
                agent.unicoreHorizontalSigma_ = 0.015;
                agent.navigation_.valid = true; agent.navigation_.haccMm = 12;
                agent.satelliteReadback_ = "satellites"; agent.satelliteReadbackMs_ = 9000;
                agent.versionReadback_ = "version"; agent.receiverInfoMs_ = 9000;
                agent.gnssVerification_ = R"({"state":"match"})"; agent.gnssVerificationMs_ = 9000;
                agent.nvsInventory_ = R"({"entries":[]})"; agent.nvsInventoryMs_ = 9000;
                agent.lastCommandResult_ = R"({"command_id":"check","status":"success","detail":"ok"})";
                agent.autoBaseProgress_ = R"({"phase":"ready_for_review","fixed_samples_only":true,"conversion_report":{"k0":0.9999}})";
                std::cout << agent.buildStatusPayload() << '\n';
            }
    for (unsigned demand = 0; demand < 2; ++demand)
        for (const char *target : {R"({"server_id":1})", R"({"server_idx":2})", R"({"server":1})",
                R"({"address":" CASTERA "})", R"({"caster_host":"casterb"})", R"({"ip":"castera"})",
                R"({"host":"casterb"})", R"({"mountpoint":"/alpha"})", R"({"mountpoint":"beta"})",
                R"({"mountpoint":"unknown"})", R"({})"})
            for (unsigned active = 0; active < 2; ++active) {
                BackendAgent agent;
                DynamicJsonDocument service(4096), payload(1024);
                deserializeJson(service, R"({"server1_enabled":true,"server2_enabled":true,"mountpoint1":"/Alpha","mountpoint2":"Beta","serverhost1":"CasterA","server2_host":"CasterB"})");
                service["stream_on_demand"] = bool(demand);
                agent.serviceConfig_.clear();
                serializeJson(service, agent.serviceConfig_);
                deserializeJson(payload, target); payload["active"] = bool(active);
                agent.applyServiceStreamCommand(payload.as<JsonObject>());
                std::cout << agent.serviceConfig_ << '\n';
            }

    for (unsigned targetId : {1U, 2U}) {
        for (const char *configuredMountpoint : {"Alpha", "/Alpha", " ///AlPhA "}) {
            for (bool explicitNull : {false, true})
            for (bool active : {false, true}) {
                BackendAgent agent;
                DynamicJsonDocument service(4096), payload(1024), result(4096);
                service["server1_enabled"] = true; service["server2_enabled"] = true;
                service["stream_on_demand"] = true;
                service["stream_active"] = !active;
                if (explicitNull) { service["server1_stream_active"] = nullptr; service["server2_stream_active"] = nullptr; }
                service[String("mountpoint") + String(targetId)] = configuredMountpoint;
                service[String("mountpoint") + String(3 - targetId)] = "Beta";
                service["active_mountpoint"] = "alpha";
                agent.serviceConfig_.clear(); serializeJson(service, agent.serviceConfig_);
                payload["active"] = active; payload["mountpoint"] = " /ALPHA ";
                agent.applyServiceStreamCommand(payload.as<JsonObject>());
                deserializeJson(result, agent.serviceConfig_);
                assert(result[String("server") + String(targetId) + "_stream_active"].is<bool>());
                assert(BackendAgent::effectiveStreamActive(result.as<JsonObjectConst>(), targetId, false) == active);
                assert(BackendAgent::effectiveStreamActive(result.as<JsonObjectConst>(), 3 - targetId, false) == !active);
                if (!active) assert(!result.containsKey("active_mountpoint"));
                for (bool globalActive : {true, false}) {
                    payload.clear(); payload["active"] = globalActive;
                    agent.applyServiceStreamCommand(payload.as<JsonObject>());
                    result.clear(); deserializeJson(result, agent.serviceConfig_);
                    for (unsigned id : {1U, 2U})
                        assert(BackendAgent::effectiveStreamActive(result.as<JsonObjectConst>(), id, false) == globalActive);
                }
            }
        }
    }
}
'''

def run(source_root, reference=False):
    header = next((source_root / 'include/functions').rglob('Backend_Agent.h')).read_text()
    header = re.sub(r'^#include.*$', '', header, flags=re.M).replace('private:', 'public:')
    status = next((source_root / 'src/functions').rglob('Backend_Agent_Status.cpp')).read_text()
    status = re.sub(r'^#include.*$', '', status, flags=re.M)
    commands = next((source_root / 'src/functions').rglob('Backend_Agent_Commands.cpp')).read_text()
    helpers = commands[commands.index('namespace {'):commands.index('void BackendAgent::recordCommandResult')] if 'namespace {' in commands else ''
    stream = 'void BackendAgent::applyServiceStreamCommand' + commands.split('void BackendAgent::applyServiceStreamCommand', 1)[1].split('\nbool BackendAgent::executeRawCommands', 1)[0]
    agent = (root / 'src/functions/backend/Backend_Agent.cpp').read_text()
    active = 'bool BackendAgent::effectiveStreamActive' + agent.split('bool BackendAgent::effectiveStreamActive', 1)[1].split('\nbool BackendAgent::mqttConnected', 1)[0]
    telemetry = 'struct GnssTelemetrySnapshot {' + snapshot + '\n};\nGnssTelemetrySnapshot getGnssTelemetrySnapshot() { GnssTelemetrySnapshot s; s.hasData = true; s.lastDataMs = s.lastGgaMs = 9000; s.satellites = 20; return s; }\n'
    status = mapped(next((source_root / 'src/functions').rglob('Backend_Agent_Status.cpp')), status, stripped_includes=True)
    active = mapped(root / 'src/functions/backend/Backend_Agent.cpp', active)
    helpers = mapped(next((source_root / 'src/functions').rglob('Backend_Agent_Commands.cpp')), helpers) if helpers else helpers
    stream = mapped(next((source_root / 'src/functions').rglob('Backend_Agent_Commands.cpp')), stream)
    with tempfile.TemporaryDirectory(prefix='esp-status-stream-') as directory:
        path = Path(directory)
        test_code = checks[:checks.index('    for (unsigned demand')] + '}\n' if reference else checks
        (path / 'check.cpp').write_text(shim + common + telemetry + header + status + active + helpers + stream + test_code)
        compile_cpp(['c++', '-std=c++17', '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined',
                        '-I' + str(root / 'include'), '-I' + str(root / '.pio/libdeps/tdm240x_dev_general/ArduinoJson/src'),
                        str(path / 'check.cpp'), '-o', str(path / 'check')], check=True)
        return [json.loads(line) for line in cpp_output([str(path / 'check')], text=True).splitlines()]

actual = run(root)
assert len(actual) == 188
for row in actual[:144]:
    assert row['serial'] == 'test' and row['version'] == 'test'
    assert row['system_info']['gnss_verification']['state'] == 'match'
    assert row['system_info']['nvs']['last_action']['command_id'] == 'check'
    assert row['auto_base_progress']['conversion_report']['k0'] == 0.9999
    if row['detected_chip_type'] == 'UNKNOWN':
        assert row['hacc'] is None
    elif row['detected_chip_type'] == 'Ublox':
        assert row['hacc'] == 12
    else:
        assert abs(row['hacc'] - 15) < 0.001
targets = [1, 2, 1, 1, 2, 1, 2, 1, 2, None, None]
for index, row in enumerate(actual[144:]):
    demand, choice, active = index // 22, (index % 22) // 2, bool(index % 2)
    assert row['stream_active'] is (active or not demand)
    target = targets[choice]
    for server in (1, 2):
        key = f'server{server}_stream_active'
        targeted = target is None or target == server
        if demand:
            assert row[key] is (active if targeted else False)
        elif targeted and active:
            assert row[key] is True
        else:
            assert key not in row

if len(sys.argv) > 1:
    # Compare status only: old stream code had dangling JSON keys (ASan failure).
    assert actual[:144] == run(Path(sys.argv[1]), reference=True), 'Status changed across refactor'
print('PASS: 144 real status cases, 44 stream commands and 72 scoped/global transitions; verification/NVS/Auto Base fields retained' + ('; status identical to before refactor' if len(sys.argv) > 1 else ''))
