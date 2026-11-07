#pragma once

#include <cstddef>
#include <cstdint>

// Whole-device lock plus independent locks for source servers 1 and 2.
struct StationLock
{
    bool all = false;
    uint8_t servers = 0;

    bool locked(uint8_t serverId = 0) const
    {
        return all || (serverId >= 1 && serverId <= 2 && (std::byte{servers} & (std::byte{1} << (serverId - 1))) != std::byte{});
    }

    bool setScoped(const int *ids, size_t count)
    {
        std::byte mask = {};
        for (size_t i = 0; i < count; ++i)
        {
            if (ids[i] < 1 || ids[i] > 2)
                return false;
            mask |= std::byte{1} << (ids[i] - 1);
        }
        all = false;
        servers = std::to_integer<uint8_t>(mask);
        return true;
    }

    void restore(bool wholeDevice, uint8_t mask)
    {
        // Corrupt persisted locks must not silently unlock an output.
        all = wholeDevice || mask > 3;
        servers = mask <= 3 ? mask : 3;
    }
};
