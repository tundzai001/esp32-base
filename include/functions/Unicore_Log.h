#pragma once
#include <cstddef>

// N4 MODE uses XOR including '#'; ASCII data logs use CRC32 excluding '#'.
bool validUnicoreLogChecksum(const char *line, size_t length);
