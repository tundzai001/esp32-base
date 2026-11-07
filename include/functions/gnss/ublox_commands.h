#pragma once
#include "functions/gnss/gnss_cmd_builder.h"

namespace UbloxCommands
{
    using GnssCmdBuilder::Command;
    using GnssCmdBuilder::CommandList;
    using GnssCmdBuilder::GnssOptions;

    Command buildOutputConfigCommand(const GnssOptions &options);
    CommandList buildBaseSurveyInCommand(uint32_t duration, float accuracy, const GnssOptions &options);
    CommandList buildBaseFixedLlaCommand(double lat, double lon, double alt, float accuracy, const GnssOptions &options);
    CommandList buildBaseRtcmOutputCommand(const String &message, const String &port, bool enabled);
    CommandList buildBaseAllRtcmOutputCommand(const String &port, bool enabled);
    CommandList buildRoverCommand();
    void routeUbloxOutput(uint8_t *data, size_t length);
}
