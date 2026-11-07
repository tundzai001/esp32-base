#pragma once
#include <cstddef>

// The backend verifies the exact per-device token; firmware checks its format.
inline bool isDeviceBearerToken(const char *token, size_t length)
{
    if (!token || length < 43 || length > 67 || token[0] != 'd' || token[1] != 't' || token[2] != '_')
        return false;
    for (size_t i = 3; i < length; ++i)
    {
        const char c = token[i];
        if (!(c >= 'A' && c <= 'Z') && !(c >= 'a' && c <= 'z') && !(c >= '0' && c <= '9') && c != '_' && c != '-')
            return false;
    }
    return true;
}
