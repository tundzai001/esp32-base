#ifndef WIFI_HANDLER_H
#define WIFI_HANDLER_H

// ============= GẮN THƯ VIỆN CẦN THIẾT =============
#include <WiFi.h>
#include "Top_Lvl_Config.h"
#include <functional>

// ================= HÀM CẤU HÌNH WI-FI =================

bool setupWiFi(const std::function<bool()> &validateConnection = {});

#endif