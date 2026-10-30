#pragma once
// Minimal host shim for the actual firmware command builder, not a receiver mock.
#include <string>
#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdint>
#include <type_traits>
inline bool isDigit(char value) { return value >= '0' && value <= '9'; }

class String : public std::string {
public:
    using std::string::string;
    String(const std::string &value) : std::string(value) {}
    template<class T, typename std::enable_if<std::is_integral<T>::value, int>::type = 0>
    String(T value) : std::string(std::to_string(value)) {}
    String(double value, unsigned precision) {
        char buffer[100];
        snprintf(buffer, sizeof(buffer), "%.*f", precision, value);
        assign(buffer);
    }
    bool isEmpty() const { return empty(); }
    int indexOf(char value) const { const auto pos = find(value); return pos == npos ? -1 : int(pos); }
    bool concat(const char *value, size_t length) { append(value, length); return true; }
    bool concat(const char *value) { append(value); return true; }
    bool concat(char value) { push_back(value); return true; }
    bool endsWith(const String &suffix) const {
        return size() >= suffix.size() && compare(size() - suffix.size(), suffix.size(), suffix) == 0;
    }
    void trim() {
        const auto start = find_first_not_of(" \r\n\t");
        *this = start == npos ? String() : String(substr(start, find_last_not_of(" \r\n\t") - start + 1));
    }
    void toUpperCase() {
        std::transform(begin(), end(), begin(), [](unsigned char c) { return std::toupper(c); });
    }
};
struct HostSerial {
    template<class... Args> void printf(const char *, Args...) {}
    void println(const char *) {}
};
inline HostSerial Serial;
