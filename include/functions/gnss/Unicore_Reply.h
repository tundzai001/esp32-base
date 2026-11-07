#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <cctype>
#include <cstring>

// UM980/UM982: $command,<command>,response: OK[*checksum]\r\n.
// Match the first command token; only an exact "OK" response succeeds.
struct UnicoreReply
{
    enum class State
    {
        Pending,
        Ok,
        Error
    };
    std::array<char, 384> line = {};
    std::array<char, 32> key = {};
    size_t used = 0;
    State state = State::Pending;

    UnicoreReply(const uint8_t *command, size_t length)
    {
        if (!command)
            return;
        // One command per exchange: a second line would have an unverified reply.
        bool ended = false;
        for (size_t j = 0; j < length; ++j)
        {
            const uint8_t c = command[j];
            if ((c < 0x20 && c != '\r' && c != '\n' && c != '\t') || c > 0x7e)
                return;
            if (c == '\r' || c == '\n')
                ended = true;
            else if (ended && !std::isspace(c))
                return;
        }
        size_t i = 0;
        size_t n = 0;
        while (i < length && std::isspace(command[i]))
            ++i;
        if (i < length && command[i] == '$')
            ++i;
        while (i < length && !std::isspace(command[i]) && command[i] != '*')
        {
            if (n + 1 >= key.size() || command[i] < 0x21 || command[i] > 0x7e)
            {
                key[0] = 0;
                return;
            }
            key[n++] = static_cast<char>(std::tolower(command[i++]));
        }
        key[n] = 0;
    }

    static bool equal(const char *text, size_t length, const char *expected)
    {
        if (length != strnlen(expected, length))
            return false;
        for (size_t i = 0; i < length; ++i)
            if (std::tolower(static_cast<unsigned char>(text[i])) !=
                std::tolower(static_cast<unsigned char>(expected[i])))
                return false;
        return true;
    }

    static int hex(char value)
    {
        if (value >= '0' && value <= '9')
            return value - '0';
        value = static_cast<char>(std::tolower(static_cast<unsigned char>(value)));
        return value >= 'a' && value <= 'f' ? value - 'a' + 10 : -1;
    }

    bool parse()
    {
        if (!key[0] || strnlen(line.data(), line.size()) < 9 || !equal(line.data(), 9, "$command,"))
            return false;
        const char *command = line.data() + 9;
        const char *comma = std::strchr(command, ',');
        if (!comma)
            return false;
        while (command < comma && std::isspace(static_cast<unsigned char>(*command)))
            ++command;
        const char *endKey = command;
        while (endKey < comma && !std::isspace(static_cast<unsigned char>(*endKey)))
            ++endKey;
        if (!equal(command, static_cast<size_t>(endKey - command), key.data()))
            return false;
        const char *response = comma + 1;
        while (std::isspace(static_cast<unsigned char>(*response)))
            ++response;
        if (strnlen(response, line.size()) < 9 || !equal(response, 9, "response:"))
            return false;
        response += 9;
        const char *end = line.data() + strnlen(line.data(), line.size());
        if (const char *star = std::strchr(response, '*'); star)
        {
            if (end - star != 3 || hex(star[1]) < 0 || hex(star[2]) < 0)
                return false;
            std::byte checksum = {};
            // N4 '$command' replies include '$' in XOR (unlike standard NMEA).
            for (const char *p = line.data(); p < star; ++p)
                checksum ^= std::byte{static_cast<unsigned char>(*p)};

            if (
                const auto expected =
                    std::byte{static_cast<unsigned char>((hex(star[1]) << 4) | hex(star[2]))};
                checksum != expected)
                return false;
            end = star;
        }
        while (response < end && std::isspace(static_cast<unsigned char>(*response)))
            ++response;
        while (end > response && std::isspace(static_cast<unsigned char>(end[-1])))
            --end;
        if (response == end)
            return false;
        state = equal(response, static_cast<size_t>(end - response), "OK") ? State::Ok : State::Error;
        return true;
    }

    bool feed(uint8_t byte)
    {
        if (byte == '$')
        {
            used = 0;
            line[used++] = '$';
            return false;
        }
        if (!used)
            return false;
        if (byte == '\r' || byte == '\n')
        {
            line[used] = 0;
            used = 0;
            return parse();
        }
        if ((byte < 0x20 && byte != '\t') || byte > 0x7e || used + 1 >= line.size())
        {
            used = 0;
            return false;
        }
        line[used++] = static_cast<char>(byte);
        return false;
    }
};
