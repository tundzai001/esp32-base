#ifndef HELPER_H
#define HELPER_H

#include "Top_Lvl_Config.h"
#include "Prog_Config.h"
#include "hardware/Connection_type.h"
#include <Arduino.h>

#include "hardware/Wifi_handler.h"
#include "hardware/Sim_handler.h"
#include "functions/mqtt/MQTT_Manager.h"

#include "functions/ntrip/ntrip_base.h"

// ================= ĐỊNH NGHĨA CÁC BIẾN TOÀN CỤC =================
extern TinyGsm modem;

class SerialCommandProcessor
{
public:
	void processPending();

private:
	static constexpr size_t MAX_COMMAND_LENGTH = 256;

	void execute(String command);

	String commandBuffer;
};

// ================= ĐỊNH NGHĨA CÁC HÀM =================
String formDeviceHealthString(int32_t signalQualityDbm, bool gnssDataOk,
							  const uint32_t *rtcmMessageCounts);

void shutdownTcpTransportBeforeRestart();

#endif
