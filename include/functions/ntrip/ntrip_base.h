#ifndef NTRIP_BASE_H
#define NTRIP_BASE_H

#include "functions/ntrip/ntrip_common.h"
#include "functions/gnss/gnss_cmd_builder.h"

int setupNTRIP();
int bootstrapUM980();
int bootstrapUblox();
GnssCmdBuilder::GnssSensorType detectGnssSensorType();
int loopNTRIP(String &rtcmData);
int connectNTRIP();
bool isNtripConnected();
void stopNtrip();
bool ntripServerConnected(uint8_t serverId);
uint32_t ntripServerBps(uint8_t serverId);
String ntripServerError(uint8_t serverId);
Client &activeNtripClient();

#endif
