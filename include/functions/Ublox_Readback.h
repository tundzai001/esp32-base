#pragma once
#include <Arduino.h>
#include "functions/Ubx_Base_Status.h"

namespace UbloxReadback {
bool readBaseStatus(UbxBaseStatus &status, String &error);
}
