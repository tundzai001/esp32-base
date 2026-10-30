#pragma once

#include <cstddef>
#include <cstdint>
#include <cctype>
#include <cstring>

// Route output logs to COM1..3, or omit COM for the current command port (0).
// Leave binary packets and other ASCII commands unchanged.
inline size_t routeUnicoreOutput(uint8_t *data, size_t length, uint8_t port) {
    if (!data || port > 3) return length;
    size_t tokenLength = 0;
    while (tokenLength < length && data[tokenLength] != ' ') ++tokenLength;
    if (tokenLength < 5 || tokenLength > 8 || tokenLength + 6 > length) return length;
    char name[9] = {};
    for (size_t i = 0; i < tokenLength; ++i) {
        name[i] = static_cast<char>(std::tolower(data[i]));
    }
    bool output = false;
    if (strncmp(name, "rtcm", 4) == 0) {
        output = true;
        for (size_t i = 4; i < tokenLength; ++i) {
            if (!std::isdigit(static_cast<unsigned char>(name[i]))) output = false;
        }
    } else if (tokenLength == 5) {
        const bool talker = strncmp(name, "gp", 2) == 0 || strncmp(name, "gn", 2) == 0
            || strncmp(name, "gl", 2) == 0 || strncmp(name, "ga", 2) == 0
            || strncmp(name, "bd", 2) == 0 || strncmp(name, "gb", 2) == 0
            || strncmp(name, "gq", 2) == 0;
        output = talker && (strcmp(name + 2, "gga") == 0 || strcmp(name + 2, "gsa") == 0
            || strcmp(name + 2, "gsv") == 0 || strcmp(name + 2, "gst") == 0);
    }
    const size_t start = tokenLength + 1;
    if (output && std::tolower(data[start]) == 'c' && std::tolower(data[start + 1]) == 'o'
        && std::tolower(data[start + 2]) == 'm' && data[start + 3] >= '1'
        && data[start + 3] <= '3' && data[start + 4] == ' ') {
        if (port == 0) {
            std::memmove(data + start, data + start + 5, length - start - 5);
            return length - 5;
        }
        data[start + 3] = static_cast<uint8_t>('0' + port);
    }
    return length;
}
