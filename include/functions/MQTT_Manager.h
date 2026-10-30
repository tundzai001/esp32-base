#ifndef MQTT_MANAGER_H
#define MQTT_MANAGER_H

// ============ GẮN CÁC THƯ VIỆN CẦN THIẾT ==============

#include <Arduino.h>
#include <PubSubClient.h>
#include "Top_Lvl_Config.h"
#include "hardware/Connection_type.h"

extern PubSubClient mqtt;
extern SemaphoreHandle_t tcpStreamMutex;

// ================= KHAI BÁO HÀM =================

// Caller owns tcpStreamMutex once networking tasks are running.
bool probeMqttTransport(bool useGsm);
int setupMQTT();
int connectMQTT();
void mqttCallback(char* topic, byte* payload, unsigned int length);
int publishHealth(const String& payload);
bool isMqttConnected();
Client& activeMqttClient();

#endif
