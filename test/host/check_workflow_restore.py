"""Exercise the real Base restore and Rover correction delivery with controlled I/O."""
from pathlib import Path
import subprocess
import tempfile
from host_coverage import compile_cpp, run_cpp, cpp_output, mapped

root = Path(__file__).resolve().parents[2]
workflow = (root / 'src/functions/backend/Backend_Agent_Workflows.cpp').read_text()
number = 'double coordinateNumber(' + workflow.split('double coordinateNumber(', 1)[1].split('// Keep audit', 1)[0]
restore = 'bool BackendAgent::restoreWorkflowBase(' + workflow.split('bool BackendAgent::restoreWorkflowBase(', 1)[1].split('\nvoid BackendAgent::runWorkflow()', 1)[0]
snapshot = workflow[workflow.index('    DynamicJsonDocument previousBase(CONFIG_DOCUMENT_SIZE);'):workflow.index('    auto restorePreviousBase = [&]() {')]
snapshot = 'bool BackendAgent::restoreSnapshot() {\n' + snapshot + '    surveyInTargetSeconds_ = 0; // Rover mode clears the live target after snapshot.\n    return restoreWorkflowBase(previousBase.as<JsonObjectConst>());\n}\n'
ntrip = (root / 'src/functions/ntrip/ntrip_rover.cpp').read_text()
rover = 'int loopNtripRover()' + ntrip.split('int loopNtripRover()', 1)[1].split('\nvoid stopNtripRover()', 1)[0]
shim = (root / 'test/host/Arduino.h').read_text().replace('#pragma once', '')
harness = r'''
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <array>
#include <vector>
#include <ArduinoJson.h>
constexpr size_t CONFIG_DOCUMENT_SIZE = 4096;
#include "functions/gnss/gnss_cmd_builder.h"
unsigned now = 10000;
uint32_t millis() { return now; }
struct BackendAgent {
    String sensorType_ = "Unicorecomm", baseStartupState_ = "survey_started", baseStartupError_, receiverBaseReadback_;
    bool reject = false, readFails = false, mismatch = false, surveyInResultPending_ = false;
    unsigned writes = 0;
    bool survey = false;
    uint32_t duration = 0;
    GnssCmdBuilder::GnssOptions restoredOptions;
    bool restoreWorkflowBase(JsonObjectConst);
    bool restoreSnapshot();
    String baseConfig_;
    uint32_t surveyInTargetSeconds_ = 120;
    bool applyBaseCoordinates(double lat, double lon, double alt) { ++writes; survey = false; return !reject && std::isfinite(lat+lon+alt); }
    bool writeGnssCommands(const GnssCmdBuilder::CommandList &commands) { ++writes; return !reject && !commands.empty(); }
    bool refreshReceiverReadback(String &) {
        receiverBaseReadback_ = mismatch ? "{\"mode_kind\":\"rover\"}" : survey ? "{\"mode_kind\":\"survey_in\"}" : "{\"mode_kind\":\"base\"}";
        return !readFails;
    }
    void markStatusDirty() {}
} *current;
namespace GnssCmdBuilder {
CommandList buildBaseSurveyInCommand(uint32_t duration, float, GnssSensorType chip, const GnssOptions &options) {
    current->survey = true; current->duration = duration; current->restoredOptions = options;
    return chip == GnssSensorType::Invalid ? CommandList{} : CommandList{Command{'m','o','d','e'}};
}
}
bool roverSessionActive = true, roverSessionAuthenticated = true;
unsigned roverLastDiagnosticMs = 0, roverBytesReceived = 0, roverBytesWritten = 0, roverPacketsReceived = 0;
const char *roverLastError = "CORS rover connection lost";
void *rtcmBufferMutex = (void*)1;
constexpr unsigned MUTEX_TIMEOUT_MS = 1500, pdTRUE = 1;
unsigned pdMS_TO_TICKS(unsigned n) { return n; }
bool xSemaphoreTake(void *, unsigned) { return true; }
unsigned releases = 0;
void xSemaphoreGive(void *) { ++releases; }
using std::min;
struct { std::array<char,16> fixStatus = {'S','I','N','G','L','E'}; } snapshot;
auto getGnssTelemetrySnapshot() { return snapshot; }
struct Client {
    std::vector<uint8_t> data;
    bool up = true;
    bool connected() { return up; }
    int available() { return data.size(); }
    int read(uint8_t *out, size_t n) { memcpy(out, data.data(), n); data.erase(data.begin(), data.begin()+n); return n; }
} client;
Client &clientFor(unsigned) { return client; }
void sendRoverGgaIfAvailable(Client &) {}
struct {
    std::vector<uint8_t> data;
    bool shortWrite = false;
    size_t write(const uint8_t *input, size_t n) { if (shortWrite) --n; data.insert(data.end(), input, input+n); return n; }
} Serial1;
struct { void feed(const uint8_t *, size_t) {} unsigned validFrames() { return 0; } } roverDatumSession;
'''
header = (root / 'include/functions/backend/Backend_Agent.h').read_text()
gate = '    bool baseStartupConfigured() const {' + header.split('    bool baseStartupConfigured() const {', 1)[1].split('    bool mqttConnected()', 1)[0]
harness = harness.replace('    void markStatusDirty() {}', gate + '    void markStatusDirty() {}')
checks = r'''
int main() {
    DynamicJsonDocument config(2048);
    deserializeJson(config, R"({"base_setup_method":"SURVEY_IN","survey_in_duration":"300","survey_in_accuracy":2,"gnss_options":{"output_mode":"rtcm_only","msm_level":7,"constellations":{"gps":true,"glo":false}}})");
    BackendAgent agent; current = &agent;
    assert(agent.restoreWorkflowBase(config.as<JsonObjectConst>()));
    assert(agent.duration == 300 && agent.baseStartupConfigured());
    assert(agent.surveyInResultPending_);
    agent.baseConfig_ = R"({"base_setup_method":"SURVEY_IN","survey_in_duration":1800})";
    assert(agent.restoreSnapshot() && agent.duration == 120);
    assert(agent.baseConfig_ == R"({"base_setup_method":"SURVEY_IN","survey_in_duration":1800})"); // Keep the UI preference unchanged.
    agent.surveyInTargetSeconds_ = 300;
    assert(agent.restoreSnapshot() && agent.duration == 300); // Never hardcode 120 for a UI-applied survey.
    agent.surveyInTargetSeconds_ = 0;
    assert(!agent.restoreSnapshot() && !agent.baseStartupConfigured()); // Unknown actual target must not fall back to 1800.
    assert(agent.restoreWorkflowBase(config.as<JsonObjectConst>()));

    assert(agent.restoredOptions.outputMode == GnssCmdBuilder::OutputMode::RtcmOnly);
    assert(agent.restoredOptions.msmLevel == GnssCmdBuilder::MsmLevel::Msm7 && !agent.restoredOptions.glo);
    for (unsigned failure = 0; failure < 3; ++failure) {
        BackendAgent failed; current = &failed;
        failed.reject = failure == 0; failed.readFails = failure == 1; failed.mismatch = failure == 2;
        assert(!failed.restoreWorkflowBase(config.as<JsonObjectConst>()));
        assert(failed.baseStartupState_ == "error" && !failed.baseStartupConfigured()); // Existing stream gate refuses publishing.
    }
    current = &agent;
    config["survey_in_duration"] = "bad";
    unsigned before = agent.writes;
    assert(!agent.restoreWorkflowBase(config.as<JsonObjectConst>()) && agent.writes == before);
    config.clear(); deserializeJson(config, R"({"base_setup_method":"FIXED_LLA","coords":{"lat":"21","lon":"105","alt":"12"}})");
    assert(agent.restoreWorkflowBase(config.as<JsonObjectConst>()));
    assert(!agent.survey && agent.baseStartupState_ == "configured");
    config.clear(); config.to<JsonObject>();
    assert(agent.restoreWorkflowBase(config.as<JsonObjectConst>()));
    assert(agent.baseStartupState_ == "awaiting_configuration" && !agent.baseStartupConfigured());
    client.data = {0xd3,0,1,0x66};
    assert(loopNtripRover() == 200);
    assert(roverBytesReceived == 4 && roverBytesWritten == 4 && Serial1.data.size() == 4);
    client.data = std::vector<uint8_t>(504, 0xd3);
    assert(loopNtripRover() == 200 && roverSessionActive && roverSessionAuthenticated);
    assert(roverBytesReceived == 508 && roverBytesWritten == 508 && Serial1.data.size() == 508);
    client.data = {0xd3,0,2,0x55}; Serial1.shortWrite = true;
    assert(loopNtripRover() == 504 && roverBytesReceived == 512 && roverBytesWritten == 511);
    assert(strcmp(roverLastError, "GNSS UART did not accept all CORS bytes") == 0 && releases == 3);
    client.up = false;
    assert(loopNtripRover() == 504 && !roverSessionAuthenticated);
}
'''
number = mapped(root / 'src/functions/backend/Backend_Agent_Workflows.cpp', number)
restore = mapped(root / 'src/functions/backend/Backend_Agent_Workflows.cpp', restore)
rover = mapped(root / 'src/functions/ntrip/ntrip_rover.cpp', rover)
with tempfile.TemporaryDirectory(prefix='esp-workflow-restore-') as directory:
    path = Path(directory)
    (path / 'Arduino.h').write_text('#pragma once\n' + shim)
    (path / 'check.cpp').write_text('#include <Arduino.h>\n' + harness + number + restore + snapshot + rover + checks)
    compile_cpp(['c++', '-std=c++17', '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined',
                    '-I' + str(path), '-I' + str(root / 'include'), '-I' + str(root / '.pio/libdeps/tdm240x_dev_general/ArduinoJson/src'),
                    str(path / 'check.cpp'), '-o', str(path / 'check')], check=True)
    run_cpp([str(path / 'check')], check=True)
print('PASS: actual 120/300s snapshot over saved 1800s, Survey-In/Fixed restore, mode verification, failure publishing gate, CORS receive/UART delivery and short writes')
