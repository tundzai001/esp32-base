#include "functions/Unicore_Log.h"

#include <cstdint>

namespace {

size_t checksumMarker(const char *line, size_t length) {
    size_t marker = length;
    while (marker > 0 && line[marker - 1] != '*') --marker;
    return marker == 0 ? length : marker - 1;
}

int hexValue(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

bool readExpectedChecksum(
    const char *line, size_t marker, size_t length, uint32_t &expected) {
    for (size_t i = marker + 1; i < length; ++i) {
        const int value = hexValue(line[i]);
        if (value < 0) return false;
        expected = (expected << 4) | static_cast<uint32_t>(value);
    }
    return true;
}

uint32_t calculateChecksum(
    const char *line, size_t marker, size_t digits) {
    uint32_t checksum = 0;
    for (size_t i = digits == 2 ? 0 : 1; i < marker; ++i) {
        checksum ^= static_cast<uint8_t>(line[i]);
        if (digits != 8) continue;
        for (uint8_t bit = 0; bit < 8; ++bit) {
            checksum = (checksum >> 1)
                ^ (checksum & 1 ? 0xedb88320U : 0U);
        }
    }
    return checksum;
}

} // namespace

bool validUnicoreLogChecksum(const char *line, size_t length) {
    if (!line || length < 4 || line[0] != '#') return false;
    const size_t star = checksumMarker(line, length);
    if (star == length) return false;
    const size_t digits = length - star - 1;
    if (digits != 2 && digits != 8) return false;
    uint32_t expected = 0;
    if (!readExpectedChecksum(line, star, length, expected)) return false;
    return calculateChecksum(line, star, digits) == expected;
}
