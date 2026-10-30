#pragma once
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

struct UbxBaseStatus {
    uint8_t mode = 0;
    bool valid = false;
    bool active = false;
    bool hasPosition = false;
    uint32_t duration = 0;
    uint32_t targetSeconds = 0;
    uint32_t observations = 0;
    double accuracy = 0;
    double latitude = 0, longitude = 0, height = 0;

    static uint32_t u32(const uint8_t *p) {
        return uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24;
    }
    static int32_t i32(const uint8_t *p) {
        const uint32_t bits = u32(p);
        int32_t value;
        std::memcpy(&value, &bits, sizeof(value));
        return value;
    }
    static int hp(uint8_t value) { return value < 128 ? value : int(value) - 256; }

    bool ecef(double x, double y, double z) {
        constexpr double a = 6378137.0, f = 1.0 / 298.257223563;
        constexpr double e2 = f * (2 - f), radToDeg = 180.0 / 3.14159265358979323846;
        const double radius = std::sqrt(x*x + y*y + z*z), p = std::hypot(x, y);
        if (!std::isfinite(radius) || radius < 5000000 || radius > 8000000) return false;
        longitude = std::atan2(y, x) * radToDeg;
        if (p < 1e-8) {
            latitude = z >= 0 ? 90 : -90;
            height = std::abs(z) - a * (1-f);
            return true;
        }
        double lat = std::atan2(z, p * (1-e2));
        for (int i = 0; i < 20; ++i) {
            const double n = a / std::sqrt(1-e2 * std::sin(lat)*std::sin(lat));
            const double next = std::atan2(z + e2*n*std::sin(lat), p);
            if (std::abs(next-lat) < 1e-15) { lat = next; break; }
            lat = next;
        }
        const double n = a / std::sqrt(1-e2 * std::sin(lat)*std::sin(lat));
        latitude = lat * radToDeg;
        height = p / std::cos(lat) - n;
        return std::isfinite(height);
    }

    bool tmode(const uint8_t *p, size_t length) {
        if (!p || length != 40 || p[0] != 0 || p[2] > 2) return false;
        mode = p[2];
        targetSeconds = u32(p+24);
        hasPosition = false;
        if (mode != 2) return true;
        if (hp(p[16]) < -99 || hp(p[16]) > 99 || hp(p[17]) < -99 || hp(p[17]) > 99
            || hp(p[18]) < -99 || hp(p[18]) > 99) return false;
        if (p[3] & 1) {
            latitude = i32(p+4)*1e-7 + hp(p[16])*1e-9;
            longitude = i32(p+8)*1e-7 + hp(p[17])*1e-9;
            height = i32(p+12)*0.01 + hp(p[18])*0.0001;
            hasPosition = std::abs(latitude) <= 90 && std::abs(longitude) <= 180;
        } else {
            hasPosition = ecef(i32(p+4)*0.01 + hp(p[16])*0.0001,
                               i32(p+8)*0.01 + hp(p[17])*0.0001,
                               i32(p+12)*0.01 + hp(p[18])*0.0001);
        }
        valid = hasPosition;
        return hasPosition;
    }

    bool svin(const uint8_t *p, size_t length) {
        if (!p || length != 40 || p[0] != 0 || p[36] > 1 || p[37] > 1) return false;
        valid = p[36] == 1; active = p[37] == 1;
        duration = u32(p+8); observations = u32(p+32); accuracy = u32(p+28)*0.0001;
        hasPosition = false;
        if (active || valid) {
            if (hp(p[24]) < -99 || hp(p[24]) > 99 || hp(p[25]) < -99 || hp(p[25]) > 99
                || hp(p[26]) < -99 || hp(p[26]) > 99) return false;
            hasPosition = ecef(i32(p+12)*0.01 + hp(p[24])*0.0001,
                               i32(p+16)*0.01 + hp(p[25])*0.0001,
                               i32(p+20)*0.01 + hp(p[26])*0.0001);
        }
        return !valid || active || hasPosition;
    }
};
