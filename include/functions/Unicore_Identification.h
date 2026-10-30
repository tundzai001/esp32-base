#pragma once
#include <cstddef>
#include <cstdint>
#include <cctype>
#include <cstring>

// Extract the ASCII identity line even when the UART also carries binary RTCM.
struct UnicoreIdentification {
    char line[512] = {};
    size_t used = 0;

    bool feed(uint8_t byte) {
        if (byte == '#') { used = 1; line[0] = '#'; return false; }
        if (!used) return false;
        if (byte == '\r' || byte == '\n') {
            line[used] = '\0';
            used = 0;
            return (std::strncmp(line, "#VERSIONA,", 10) == 0 || std::strncmp(line, "#VERSION,", 9) == 0)
                && (std::strstr(line, "\"UM9") || std::strstr(line, "\"UB9")
                    || std::strstr(line, "\"UB4") || std::strstr(line, "UNICORE"));
        }
        if (byte < 0x20 || byte > 0x7e || used + 1 >= sizeof(line)) {
            used = 0;
            return false;
        }
        line[used++] = static_cast<char>(std::toupper(byte));
        return false;
    }
};
