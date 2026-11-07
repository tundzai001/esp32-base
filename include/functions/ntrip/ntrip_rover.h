#ifndef NTRIP_ROVER_H
#define NTRIP_ROVER_H

#include "functions/ntrip/ntrip_common.h"

bool startNtripRover(const String &host, uint16_t port, const String &username,
                     const String &password, String mountpoint,
                     uint8_t version = 1);
int loopNtripRover();
void stopNtripRover();
bool ntripRoverActive();
uint32_t ntripRoverBytesReceived();
uint32_t ntripRoverPacketsReceived();

#endif
