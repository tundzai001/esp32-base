#ifndef RTCM_RECEIVER_H
#define RTCM_RECEIVER_H

#include "Top_Lvl_Config.h"
#include "Prog_Config.h"
#include "WString.h"
#include <cstdint>

inline constexpr size_t RTCM_SUPPORTED_MESSAGE_TYPE_COUNT = 10;

struct RtcmMessageCounts {
	uint32_t values[RTCM_SUPPORTED_MESSAGE_TYPE_COUNT] = {};
};

// Snapshot of NMEA-derived position/quality and UART/frame counters.
// RTCM frames are counted and forwarded without CRC or message-type decoding.
struct GnssTelemetrySnapshot {
	bool hasData = false;
	uint32_t lastDataMs = 0;
	uint32_t lastGgaMs = 0;
	uint64_t bytesRead = 0;
	uint32_t rtcmFrames = 0;
	uint32_t nmeaSentences = 0;
	uint32_t ggaCount = 0;
	uint32_t gsaCount = 0;
	uint32_t gsvCount = 0;
	uint32_t gstCount = 0;
    uint32_t snrSamples = 0;
    uint32_t latestGsvSamples = 0;
    uint32_t lastGsvMs = 0;
    uint32_t lastGstMs = 0;
    double latitude = 0.0;
	double longitude = 0.0;
	double altitude = 0.0;
	float hdop = 0.0F;
    float averageSnr = 0.0F;
    float latestAverageSnr = 0.0F;
    float gstSemiMajor = 0.0F;
    float gstSemiMinor = 0.0F;
    float gstHacc = 0.0F;
    uint8_t satellites = 0;
    char fixStatus[16] = "NO_FIX";
    char lastGga[128] = {};
};

String receiveRtcmFromGnss();
// Caller owns gnssSerialMutex and rtcmBufferMutex; preserve bytes read by queries.
void bufferGnssQueryByte(uint8_t value);
RtcmMessageCounts consumeRtcmMessageCounts();
GnssTelemetrySnapshot getGnssTelemetrySnapshot();
uint32_t getRtcmInputBps();

#endif
