#include "functions/MqttWrite.h"
#include <algorithm>
#include <cassert>
#include <cstring>
#include <vector>

int main() {
    std::vector<uint8_t> packet(5411);
    for (size_t i = 0; i < packet.size(); ++i) packet[i] = static_cast<uint8_t>(i);
    std::vector<uint8_t> received;
    size_t calls = 0;
    auto fullWrite = [&](const uint8_t* data, size_t count) {
        assert(count <= 1024);
        received.insert(received.end(), data, data + count);
        ++calls;
        return count;
    };
    assert(writeMqttChunks(packet.data(), packet.size(), fullWrite) == packet.size());
    assert(received == packet);
    assert(calls == 6);
    calls = 0;
    assert(writeMqttChunks(packet.data(), 0, fullWrite) == 0);
    assert(calls == 0);
    assert(writeMqttChunks(packet.data(), 20, fullWrite) == 20);
    calls = 0;
    const auto shortWrite = [&](const uint8_t*, size_t count) {
        return ++calls == 2 ? size_t(17) : count;
    };
    assert(writeMqttChunks(packet.data(), packet.size(), shortWrite) == 1041);
    assert(calls == 2); // No replay or retry on a damaged MQTT stream.
    calls = 0;
    assert(writeMqttChunks(packet.data(), packet.size(), [&](const uint8_t*, size_t) {
        ++calls;
        return size_t(0);
    }) == 0);
    assert(calls == 1);
}
