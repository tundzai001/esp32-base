#pragma once

#include <stddef.h>
#include <stdint.h>

// Keep each modem send below its TCP command payload limit. A short write
// ends this packet; its caller must reconnect instead of replaying its prefix.
template <typename Write>
size_t writeMqttChunks(const uint8_t* data, size_t size, Write write) {
    size_t sent = 0;
    while (sent < size) {
        const size_t chunk = size - sent > 1024 ? 1024 : size - sent;
        const size_t written = write(data + sent, chunk);
        if (written > chunk) return sent;
        sent += written;
        if (written != chunk) break;
    }
    return sent;
}
