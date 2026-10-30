#include "functions/gnss_cmd_builder.h"
#include "functions/ublox_commands.h"
#include "functions/unicore_commands.h"
#include "Prog_Config.h"
#include <array>
#include <algorithm>
#include <cstring>

namespace GnssCmdBuilder {

    inline bool isValidPort(const String &port) {
        return port == "UART1" || port == "UART2" || port == "USB";
    }

    GnssOptions normalizeGnssOptions(GnssOptions options)
    {
        std::vector<String> ports;
        for (String port : options.ports)
        {
            port.trim();
            port.toUpperCase();
            if (isValidPort(port) && std::find(ports.begin(), ports.end(), port) == ports.end())
                ports.push_back(port);
        }
        options.ports = ports.empty() ? std::vector<String>{"USB"} : ports;
        if (!options.gps && !options.glo && !options.gal && !options.bds && !options.qzss && !options.sbas)
            options.gps = true;
        if (options.outputMode != OutputMode::RtcmOnly && options.msmLevel == MsmLevel::Both)
            options.msmLevel = MsmLevel::Msm4;
        return options;
    }

    GnssSensorType sensorTypeFromString(const String &sensorType)
    {
        String normalized = sensorType;
        normalized.trim();
        normalized.toUpperCase();
        if (normalized == "INVALID")
            return GnssSensorType::Invalid;
        if (normalized == "UBLOX")
            return GnssSensorType::Ublox;
        if (normalized == "UNICORE" || normalized == "UNICORECOMM")
            return GnssSensorType::Unicore;
        return GnssSensorType::Invalid;
    }

    Command commandListToBytes(const CommandList &commands)
    {
        size_t totalSize = 0;
        for (const Command &command : commands)
            totalSize += command.size();

        Command bytes;
        bytes.reserve(totalSize);
        for (const Command &command : commands)
            bytes.insert(bytes.end(), command.begin(), command.end());

        return bytes;
    }

    CommandList buildBaseSurveyInCommand(uint32_t duration, float accuracy,
                                         GnssSensorType sensorType, const GnssOptions &options)
    {
        if (sensorType == GnssSensorType::Ublox) return UbloxCommands::buildBaseSurveyInCommand(duration, accuracy, options);
        if (sensorType == GnssSensorType::Unicore) return UnicoreCommands::buildBaseSurveyInCommand(duration, accuracy, options);
        return {};
    }

    CommandList buildBaseFixedLlaCommand(double lat, double lon, double alt, float accuracy,
                                         GnssSensorType sensorType, const GnssOptions &options)
    {
        if (sensorType == GnssSensorType::Ublox) return UbloxCommands::buildBaseFixedLlaCommand(lat, lon, alt, accuracy, options);
        if (sensorType == GnssSensorType::Unicore) return UnicoreCommands::buildBaseFixedLlaCommand(lat, lon, alt, accuracy, options);
        return {};
    }

    CommandList buildBaseRtcmOutputCommand(const String &message, const String &port, bool enabled,
                                           GnssSensorType sensorType)
    {
        if (sensorType == GnssSensorType::Ublox) return UbloxCommands::buildBaseRtcmOutputCommand(message, port, enabled);
        if (sensorType == GnssSensorType::Unicore) return UnicoreCommands::buildBaseRtcmOutputCommand(message, port, enabled);
        return {};
    }

    CommandList buildBaseAllRtcmOutputCommand(const String &port, bool enabled,
                                              GnssSensorType sensorType)
    {
        if (sensorType == GnssSensorType::Ublox) return UbloxCommands::buildBaseAllRtcmOutputCommand(port, enabled);
        if (sensorType == GnssSensorType::Unicore) return UnicoreCommands::buildBaseAllRtcmOutputCommand(port, enabled);
        return {};
    }

    CommandList buildRoverCommand(GnssSensorType type) {
        if (type == GnssSensorType::Ublox) return UbloxCommands::buildRoverCommand();
        if (type == GnssSensorType::Unicore) return UnicoreCommands::buildRoverCommand();
        return {};
    }
    Command buildUbloxOutputConfigCommand(const GnssOptions &options) {
        return UbloxCommands::buildOutputConfigCommand(options);
    }
    void routeUbloxOutput(uint8_t *data, size_t length) { UbloxCommands::routeUbloxOutput(data, length); }
#if PROGRAM_DEBUG
    String debugCommand(const Command &command)
    {
        String result;
        for (uint8_t byte : command)
        {
            char hex[4];
            snprintf(hex, sizeof(hex), "%02x", byte);
            if (!result.isEmpty())
                result += ' ';
            result += hex;
        }
        return result;
    }
#endif
}
