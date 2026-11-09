"""Run the actual GGA telemetry parser, including ellipsoid height and invalid epochs."""
from pathlib import Path
import subprocess
import tempfile
from host_coverage import compile_cpp, run_cpp, cpp_output, mapped

root = Path(__file__).resolve().parents[2]
source = (root / 'src/functions/ntrip/RTCM_Receiver.cpp').read_text()
functions = 'double nmeaCoordinate(' + source.split('double nmeaCoordinate(', 1)[1].split('\nvoid resetRtcmParser', 1)[0]
header = (root / 'include/functions/ntrip/RTCM_Receiver.h').read_text()
telemetry = 'struct GnssTelemetrySnapshot {' + header.split('struct GnssTelemetrySnapshot {', 1)[1].split('\n};', 1)[0] + '\n};'
shim = (root / 'test/host/Arduino.h').read_text().replace('#pragma once', '')
shim = shim.replace('    void trim() {', '    bool startsWith(const String &value) const { return compare(0, value.size(), value) == 0; }\n    void trim() {')
harness = r'''
#include "functions/backend/Base_Sample.h"
#include <array>
#include <cassert>
#include <cmath>
using std::min;
unsigned now = 1000;
unsigned millis() { return now; }
int constrain(int value, int low, int high) { return std::max(low, std::min(value, high)); }
#define portENTER_CRITICAL(x) ((void)0)
#define portEXIT_CRITICAL(x) ((void)0)
constexpr size_t MAX_NMEA_LENGTH = 512;
struct {
    void setDetectedChipIfUnknown(const String &) {}
    void enqueueNmea(const uint8_t *, size_t) {}
} backendAgent;
'''
checks = r'''
int main() {
    parseNmeaSentence("$GNGGA,010000,2105.0994,N,10547.19936,E,4,20,0.7,20.5000,M,-28.5624,M,,*00");
    assert(telemetry.ggaCount == 1 && telemetry.satellites == 20);
    assert(strcmp(telemetry.fixStatus.data(), "RTK_FIXED") == 0);
    assert(fabs(telemetry.latitude - 21.08499) < 1e-9);
    assert(fabs(telemetry.longitude - 105.786656) < 1e-9);
    assert(fabs(telemetry.altitude + 8.0624) < 1e-9);
    assert(telemetry.lastGgaMs == now);
    parseNmeaSentence("$GNGSA,A,3,,,,,,,,,,,,,1.0,0.7,0.8*00");
    assert(strcmp(telemetry.fixStatus.data(), "RTK_FIXED") == 0);
    parseNmeaSentence("$GNGGA,010001,2105.0994,N,10547.19936,E,4,20,0.7,20.5,M,,M,,*00");
    assert(std::isnan(telemetry.altitude)); // Missing N is unknown, never zero.
    parseNmeaSentence("$GNGGA,010002,,,,,4,20,0.7,20.5,M,-28.5624,M,,*00");
    assert(std::isnan(telemetry.latitude) && std::isnan(telemetry.longitude));
    assert(std::isnan(telemetry.altitude)); // New epoch cannot reuse previous coordinates.
}
'''
functions = mapped(root / 'src/functions/ntrip/RTCM_Receiver.cpp', functions)
with tempfile.TemporaryDirectory(prefix='esp-gga-') as directory:
    path = Path(directory)
    (path / 'check.cpp').write_text(shim + harness + telemetry + '\nGnssTelemetrySnapshot telemetry;\n' + functions + checks)
    compile_cpp(['c++', '-std=c++17', '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined',
                    '-I' + str(root / 'include'), str(path / 'check.cpp'), '-o', str(path / 'check')], check=True)
    run_cpp([str(path / 'check')], check=True)
print('PASS: real GGA telemetry parses RTK Fixed LLH; h=H+N; missing height/coordinates never reuse a prior sample')
