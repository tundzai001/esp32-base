#pragma once
#include "functions/gnss_cmd_builder.h"

namespace UnicoreCommands {
using namespace GnssCmdBuilder;
CommandList buildBaseSurveyInCommand(uint32_t duration, float accuracy, const GnssOptions &options);
CommandList buildBaseFixedLlaCommand(double lat, double lon, double alt, float accuracy, const GnssOptions &options);
CommandList buildBaseRtcmOutputCommand(const String &message, const String &port, bool enabled);
CommandList buildBaseAllRtcmOutputCommand(const String &port, bool enabled);
CommandList buildRoverCommand();
CommandList buildEphemerisOutputCommands(const String &port);
}
