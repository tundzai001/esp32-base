#pragma once
#include "functions/gnss/gnss_cmd_builder.h"

namespace UnicoreCommands
{
    GnssCmdBuilder::CommandList buildBaseSurveyInCommand(
        uint32_t duration, float accuracy, const GnssCmdBuilder::GnssOptions &options);
    GnssCmdBuilder::CommandList buildBaseFixedLlaCommand(double lat, double lon,
                                                         double alt, float accuracy, const GnssCmdBuilder::GnssOptions &options);
    GnssCmdBuilder::CommandList buildBaseRtcmOutputCommand(const String &message,
                                                           const String &port, bool enabled);
    GnssCmdBuilder::CommandList buildBaseAllRtcmOutputCommand(
        const String &port, bool enabled);
    GnssCmdBuilder::CommandList buildRoverCommand();
    GnssCmdBuilder::CommandList buildEphemerisOutputCommands(const String &port);
}
