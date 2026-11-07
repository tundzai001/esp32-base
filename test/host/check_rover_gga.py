"""Run the firmware's GGA sender with a host clock and socket; no hardware needed."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
source = (root / "src/functions/ntrip/NTRIP_Handler_IP.cpp").read_text()
sender = source.split("void sendRoverGgaIfAvailable(", 1)[1].split("\nbool connectEndpoint(", 1)[0]
harness = r'''
#include <cassert>
#include <cstdint>
#include <string>
#include <vector>
using String = std::string;
uint32_t now = 10000, roverLastGgaMs = 0;
uint32_t millis() { return now; }
struct Snapshot { const char *lastGga = ""; uint32_t lastGgaMs = 0; } snapshot;
using GnssTelemetrySnapshot = Snapshot;
Snapshot getGnssTelemetrySnapshot() { return snapshot; }
struct {
    std::vector<std::string> logs;
    void println(const char *line) { logs.emplace_back(line); }
    void printf(const char *, const char *) { logs.emplace_back("sent"); }
} Serial;
struct Client {
    std::string sent;
    bool fail = false;
    unsigned writes = 0;
    size_t print(const String &line) {
        ++writes;
        sent = line;
        return fail ? 0 : line.length();
    }
};
'''
checks = r'''
int main() {
    Client client;
    sendRoverGgaIfAvailable(client, true);
    assert(client.writes == 0 && Serial.logs.size() == 1);
    sendRoverGgaIfAvailable(client);
    assert(Serial.logs.size() == 1); // Do not flood missing-GGA logs.
    now += 5000;
    snapshot = {"$GNGGA,test", now - 3001};
    sendRoverGgaIfAvailable(client);
    assert(client.writes == 0); // Never send stale receiver data.
    now += 5000;
    snapshot.lastGgaMs = now;
    sendRoverGgaIfAvailable(client);
    assert(client.writes == 1 && client.sent == "$GNGGA,test\r\n");
    assert(Serial.logs.back() == "sent");
    sendRoverGgaIfAvailable(client);
    assert(client.writes == 1); // Respect the five-second interval.
    now += 5000;
    snapshot.lastGgaMs = now;
    client.fail = true;
    sendRoverGgaIfAvailable(client);
    assert(client.writes == 2 && Serial.logs.back() != "sent");
}
'''
with tempfile.TemporaryDirectory(prefix="esp32-gga-") as directory:
    path = Path(directory)
    (path / "check.cpp").write_text(harness + "void sendRoverGgaIfAvailable(" + sender + checks)
    subprocess.run(["c++", "-std=c++17", "-fsanitize=address,undefined",
                    str(path / "check.cpp"), "-o", str(path / "check")], check=True)
    subprocess.run([str(path / "check")], check=True)
print("PASS: missing/stale GGA, CRLF forwarding, rate limit, socket write failure")
