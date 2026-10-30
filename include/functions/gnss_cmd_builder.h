#ifndef GNSS_COMMAND_BUILDER_H
#define GNSS_COMMAND_BUILDER_H

#include <Arduino.h>
#include "Top_Lvl_Config.h"
#include <cstdint>
#include <vector>

namespace GnssCmdBuilder
{

    using Command = std::vector<uint8_t>;
    using CommandList = std::vector<Command>;

    enum class GnssSensorType
    {
        Invalid,
        Ublox,
        Unicore
    };

    enum class OutputMode
    {
        DiagnosticsRtcm,
        RtcmOnly
    };
    enum class MsmLevel : uint8_t
    {
        Msm4 = 4,
        Msm7 = 7,
        Both = 0
    };

    struct GnssOptions
    {
        bool gps = true;
        bool glo = true;
        bool gal = true;
        bool bds = true;
        bool qzss = false;
        bool sbas = false;
        OutputMode outputMode = OutputMode::DiagnosticsRtcm;
        MsmLevel msmLevel = MsmLevel::Msm4;
        std::vector<String> ports = {"USB"};
    };

    GnssOptions normalizeGnssOptions(GnssOptions options = {});
    GnssSensorType sensorTypeFromString(const String &sensorType);
    Command buildUbloxOutputConfigCommand(const GnssOptions &options = {});
    CommandList buildBaseSurveyInCommand(uint32_t duration,
                                         float accuracy, GnssSensorType sensorType,
                                         const GnssOptions &options = {});
    CommandList buildBaseFixedLlaCommand(double lat, double lon,
                                         double alt, float accuracy,
                                         GnssSensorType sensorType,
                                         const GnssOptions &options = {});
    CommandList buildBaseRtcmOutputCommand(const String &message,
                                           const String &port, bool enabled,
                                           GnssSensorType sensorType);
    CommandList buildBaseAllRtcmOutputCommand(const String &port, bool enabled,
                                              GnssSensorType sensorType);
    CommandList buildRoverCommand(GnssSensorType sensorType);
    void routeUbloxOutput(uint8_t *data, size_t length);
    Command commandListToBytes(const CommandList &commands);

    #if PROGRAM_DEBUG
    String debugCommand(const Command &command);
    #endif

} // namespace GnssCmdBuilder

#endif // GNSS_COMMAND_BUILDER_H
