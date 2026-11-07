"""Exercise the actual mixed UART stream parser with interleaved UBX/RTCM/NMEA."""
from pathlib import Path
import subprocess
import sys
import tempfile

root = Path(__file__).resolve().parents[2]
source = Path(sys.argv[1]).read_text() if len(sys.argv) > 1 else (root / "src/functions/ntrip/RTCM_Receiver.cpp").read_text()
parser = source.split("void resetRtcmParser()", 1)[1].split("\n} // namespace", 1)[0]
harness = r'''
#include <cassert>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
struct String : std::string {
    using std::string::string;
    void concat(const char *data, size_t length) { append(data, length); }
};
constexpr size_t MAX_NMEA_LENGTH = 512;
String nmeaBuffer;
uint8_t rtcmFrame[1029] = {}, ubxHeader[6] = {};
size_t rtcmLength = 0, rtcmExpectedLength = 0, ubxHeaderLength = 0, ubxBytesRemaining = 0;
unsigned sentences = 0;
void parseNmeaSentence(const String &line) {
    assert(line.find("$GNGGA,real") == 0);
    ++sentences;
}
int telemetryMux = 0;
#define portENTER_CRITICAL(x) ((void)0)
#define portEXIT_CRITICAL(x) ((void)0)
struct { unsigned rtcmFrames = 0; bool hasData = false; unsigned lastDataMs = 0; } telemetry;
unsigned millis() { return 1; }
void noteRtcmBytes(size_t) {}
struct {
    std::vector<std::string> frames;
    void enqueueRtcm(const uint8_t *data, size_t length) {
        frames.emplace_back(reinterpret_cast<const char *>(data), length);
    }
} backendAgent;
'''
checks = r'''
int main() {
    String output;
    const auto feed = [&](const std::string &bytes) {
        for (unsigned char byte : bytes) consumeByte(byte, output);
    };
    const std::string gga = "$GNGGA,real\r\n";
    // A valid UBX binary payload contains a false RTCM header and NMEA start.
    std::string ubx("\xB5\x62\x01\x07\x08\x00", 6);
    ubx.append("\xD3\x00\xF0$fake", 8);
    uint8_t a = 0, b = 0;
    for (size_t i = 2; i < ubx.size(); ++i) { a += uint8_t(ubx[i]); b += a; }
    ubx += char(a); ubx += char(b);
    // An RTCM payload may contain all the other protocol markers too.
    std::string rtcm("\xD3\x00\x08", 3);
    rtcm.append("\xB5\x62$fake\n", 8);
    rtcm.append("\0\0\0", 3); // CRC is forwarded opaquely by this parser.
    for (unsigned i = 0; i < 30; ++i) {
        feed(ubx); feed(rtcm); feed(gga);
    }
    assert(sentences == 30 && backendAgent.frames.size() == 30);
    for (const auto &frame : backendAgent.frames) assert(frame == rtcm);
    feed(std::string("\xD3\xFF", 2)); feed(gga);
    assert(sentences == 31); // Invalid RTCM reserved bits cannot swallow GGA.
    feed("$truncated"); feed(ubx); feed(gga);
    assert(sentences == 32); // Binary traffic ends a malformed NMEA line.
}
'''
with tempfile.TemporaryDirectory(prefix="gnss-stream-") as directory:
    path = Path(directory)
    (path / "check.cpp").write_text(harness + "void resetRtcmParser()" + parser + checks)
    subprocess.run(["c++", "-std=c++17", "-fsanitize=address,undefined",
                    str(path / "check.cpp"), "-o", str(path / "check")], check=True)
    subprocess.run([str(path / "check")], check=True)
print("PASS: 30 interleaved GGA/UBX/RTCM cycles, binary payload markers, malformed input recovery")
