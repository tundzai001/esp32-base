#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <cctype>
#include <cstring>

// Extract the ASCII identity line even when the UART also carries binary RTCM.
struct UnicoreIdentification
{
    std::array<char, 512> line = {};
    size_t used = 0;

    bool feed(uint8_t byte)
    {
        if (byte == '#')
        {
            used = 1;
            line[0] = '#';
            return false;
        }
        if (!used)
            return false;
        if (byte == '\r' || byte == '\n')
        {
            line[used] = '\0';
            used = 0;
            return (std::strncmp(line.data(), "#VERSIONA,", 10) == 0 || std::strncmp(line.data(), "#VERSION,", 9) == 0) && (std::strstr(line.data(), "\"UM9") || std::strstr(line.data(), "\"UB9") || std::strstr(line.data(), "\"UB4") || std::strstr(line.data(), "UNICORE"));
        }
        if (byte < 0x20 || byte > 0x7e || used + 1 >= line.size())
        {
            used = 0;
            return false;
        }
        line[used++] = static_cast<char>(std::toupper(byte));
        return false;
    }
};
