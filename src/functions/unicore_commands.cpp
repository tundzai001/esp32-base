#include "functions/gnss_cmd_builder.h"
#include "functions/unicore_commands.h"
#include "Prog_Config.h"
#include <array>
#include <algorithm>
#include <cstring>

namespace UnicoreCommands {
using namespace GnssCmdBuilder;

    inline bool isValidUnicorePort(const String &port)
    {
        return port == "COM1" || port == "COM2" || port == "COM3" || port == "CURRENT";
    }

    inline String outputPortArgument(const String &port) {
        return port.isEmpty() || port == "CURRENT" ? String() : port + " ";
    }

    inline String configuredOutputPort() {
        return GNSS_UNICORE_PORT == 0 ? String() : "com" + String(GNSS_UNICORE_PORT);
    }

    inline Command asciiCommand(const String &text)
    {
        return Command(text.begin(), text.end());
    }

    inline Command delayCommand(uint16_t delayMs) {
        return asciiCommand("$DELAY_" + String(delayMs) + "$");
    }

    void appendUnicoreOutputCommands(CommandList &commands, const String &port, const GnssOptions &options)
    {
        const GnssOptions o = normalizeGnssOptions(options);

        struct Messages
        {
            bool enabled;
            const char *const *items;
            size_t count;
        };

        static constexpr std::array<const char*, 4> GPS = {
            {"gpgga", "gpgsa", "gpgsv", "gpgst"}
        };

        static constexpr std::array<const char*, 3> GLO = {
            {"glgga", "glgsa", "glgsv"}
        };

        static constexpr std::array<const char*, 3> GAL = {
            {"gagga", "gagsa", "gagsv"}
        };

        static constexpr std::array<const char*, 3> BDS = {
            {"bdgga", "bdgsa", "bdgsv"}
        };

        const std::array<Messages, 4> nmea = {{
            {o.gps, GPS.data(), GPS.size()},
            {o.glo, GLO.data(), GLO.size()},
            {o.gal, GAL.data(), GAL.size()},
            {o.bds, BDS.data(), BDS.size()},
        }};

        const auto appendNmeaGroup = [&](const Messages &group) {
            if (!group.enabled)
                return;

            for (size_t i = 0; i < group.count; ++i)
            {
                commands.push_back(asciiCommand(String(group.items[i]) + " " + outputPortArgument(port) + "1\r\n"));
                commands.push_back(delayCommand(200));
            }
        };

        if (o.outputMode != OutputMode::RtcmOnly)
            for (const auto &group : nmea)
                appendNmeaGroup(group);

        std::vector<String> rtcm = {"1006", "1033"};
        auto add = [&](bool enabled, const std::array<const char *, 3> &messages)
        {
            if (!enabled)
                return;
            for (const char *item : messages)
            {
                const String message(item);
                const bool ephemeris = message == "1019" || message == "1020" || message == "1042" || message == "1044" || message == "1045";
                const bool matches = (o.msmLevel == MsmLevel::Both && (message.endsWith("4") || message.endsWith("7"))) || message.endsWith(String(static_cast<uint8_t>(o.msmLevel)));
                if (ephemeris || matches)
                    rtcm.push_back(message);
            }
        };
        static constexpr std::array<const char *, 3> GPS_R = {
            {"1074", "1077", "1019"}
        };
        static constexpr std::array<const char *, 3> GLO_R = {
            {"1084", "1087", "1020"}
        };
        static constexpr std::array<const char *, 3> GAL_R = {
            {"1094", "1097", "1045"}
        };
        static constexpr std::array<const char *, 3> BDS_R = {
            {"1124", "1127", "1042"}
        };
        static constexpr std::array<const char *, 3> QZSS_R = {
            {"1114", "1117", "1044"}
        };

        add(o.gps, GPS_R);
        add(o.glo, GLO_R);
        add(o.gal, GAL_R);
        add(o.bds, BDS_R);
        add(o.qzss, QZSS_R);
        for (size_t i = 0; i < rtcm.size(); ++i)
            if (std::find(rtcm.begin(), rtcm.begin() + i, rtcm[i]) == rtcm.begin() + i)
            {
                commands.push_back(asciiCommand("rtcm" + rtcm[i] + " " + outputPortArgument(port) + "1\r\n"));
                commands.push_back(delayCommand(200));
            }
    }

    CommandList buildEphemerisOutputCommands(const String &port)
    {
        String normalizedPort = port;
        normalizedPort.trim();
        normalizedPort.toUpperCase();
        if (!isValidUnicorePort(normalizedPort)) return {};
        CommandList commands;
        for (const char *message : {"1019", "1020", "1042", "1044", "1045"})
            commands.push_back(asciiCommand("rtcm" + String(message) + " " + outputPortArgument(normalizedPort) + "1\r\n"));
        return commands;
    }

    CommandList buildBaseSurveyInCommand(uint32_t duration, float accuracy, const GnssOptions &options) {
        CommandList commands;
        commands.push_back(asciiCommand("unlogall\r\n"));
        commands.push_back(delayCommand(1000));
        commands.push_back(asciiCommand("config undulation 0.0\r\n"));
        const String cmd = "mode base time " + String(duration) + " 0\r\n";
        Serial.printf("Survey-In command: %s", cmd.c_str());
        commands.push_back(asciiCommand(cmd));
        commands.push_back(delayCommand(2000));
        const String port = configuredOutputPort();
        Serial.printf("Configuring Unicore output messages on %s\n", port.isEmpty() ? "current command port" : port.c_str());
        appendUnicoreOutputCommands(commands, port, options);
        commands.push_back(asciiCommand("saveconfig\r\n"));

        return commands;
    }

    CommandList buildBaseFixedLlaCommand(double lat, double lon, double alt, float accuracy, const GnssOptions &options) {
        CommandList commands;
        commands.push_back(asciiCommand("unlogall\r\n"));
        commands.push_back(delayCommand(1000));
        const String cmd = "mode base " + String(lat, 10) + " " + String(lon, 10) + " " + String(alt, 4) + "\r\n";
        Serial.printf("Fixed position command: %s", cmd.c_str());
        commands.push_back(asciiCommand(cmd));
        commands.push_back(delayCommand(2000));
        const String port = configuredOutputPort();
        Serial.printf("Configuring Unicore output messages on %s\n", port.isEmpty() ? "current command port" : port.c_str());
        appendUnicoreOutputCommands(commands, port, options);
        commands.push_back(asciiCommand("saveconfig\r\n"));

        return commands;
    }

    CommandList buildBaseRtcmOutputCommand(const String &message, const String &port, bool enabled) {

        String normalizedMessage = message;
        String normalizedPort = port;
        normalizedMessage.trim();
        normalizedMessage.toUpperCase();
        normalizedPort.trim();
        normalizedPort.toUpperCase();

        CommandList commands;
        if (!isValidUnicorePort(normalizedPort) || normalizedMessage.length() != 4) return commands;
        for (uint8_t i = 0; i < normalizedMessage.length(); ++i)
            if (!isDigit(normalizedMessage[i]))
                return commands;

        commands.push_back(asciiCommand("rtcm" + normalizedMessage + " " + outputPortArgument(normalizedPort)
                                        + String(enabled ? 1 : 0) + "\r\n"));
        commands.push_back(asciiCommand("saveconfig\r\n"));

        return commands;
    }

    CommandList buildBaseAllRtcmOutputCommand(const String &port, bool enabled) {
        String normalizedPort = port;
        normalizedPort.trim();
        normalizedPort.toUpperCase();
        if (!isValidUnicorePort(normalizedPort)) return {};

        static constexpr std::array<const char *, 17> RTCM = {
            {"1006", "1033", "1074", "1124", "1084", "1094", "1114", "1077", "1127",
             "1087", "1097", "1117", "1042", "1019", "1020", "1045", "1044"}
        };
        CommandList commands;
        for (const char *message : RTCM)
            commands.push_back(asciiCommand("rtcm" + String(message) + " " + outputPortArgument(normalizedPort)
                                            + String(enabled ? 1 : 0) + "\r\n"));
        commands.push_back(asciiCommand("saveconfig\r\n"));
        return commands;
    }

    CommandList buildRoverCommand() {
        const String port = outputPortArgument(configuredOutputPort());
        return {asciiCommand("mode rover\r\n"),
                asciiCommand("gpgga " + port + "1\r\n"),
                asciiCommand("gpgst " + port + "1\r\n"),
                asciiCommand("saveconfig\r\n")};

    }
}
