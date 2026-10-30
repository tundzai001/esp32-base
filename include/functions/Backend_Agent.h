#ifndef BACKEND_AGENT_H
#define BACKEND_AGENT_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include "functions/Station_Lock.h"
#include "functions/gnss_cmd_builder.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

class BackendAgentWebsocket;

// Device agent for backend commands, status, provisioning and base workflows.
// Receiver coordinates come from NMEA or Unicore readback; RTCM is forwarded.
class BackendAgent {
public:
    void begin();
    bool initializeBaseAtBoot(GnssCmdBuilder::GnssSensorType detected);
    void loop();
    bool detectGnssChip(Stream &gnssStream, bool force = false);
    bool applyRoverMode();
    void setDetectedChipIfUnknown(const String &chipType);
    const String &sensorType() const { return sensorType_; }

    void onMqttConnected();
    void onMqttDisconnected();
    void onNetworkChanged();
    void onMqttMessage(const char *topic, const uint8_t *payload, size_t length);

    void enqueueNmea(const uint8_t *data, size_t length);
    void enqueueRtcm(const uint8_t *data, size_t length);

    void publishStatus(bool force = false);
    void setAgentState(const String &state);
    String lwtPayload() const;

    const String &serial() const { return serialNumber_; }
    bool isLocked() const { return stationLock_.locked(); }
    bool isServerLocked(uint8_t serverId) const { return stationLock_.locked(serverId); }
    bool isProvisioned() const { return provisioned_; }
    bool licenseValid() const { return licenseValid_; }
    bool baseStartupConfigured() const {
        return baseStartupState_ == "fixed_restored" || baseStartupState_ == "survey_started"
            || baseStartupState_ == "preserved"
            || baseStartupState_ == "configured";
    }
    bool mqttConnected() const;
    bool websocketConnected() const;
    bool controlPlaneFailOpen() const { return controlPlaneFailOpen_; }
    bool workflowActive() const { return workflowTask_ != nullptr; }

    String serviceConfigJson() const;
    String baseConfigJson() const;
    String licenseToken() const;

    // Resolve per-server stream activation from service settings and fail-open state.
    static bool effectiveStreamActive(const JsonObjectConst &service,
                                      uint8_t serverId,
                                      bool failOpen);

private:
    String serialNumber_;
    String agentState_ = "initializing";
    String deviceName_;
    String lastCommandResult_ = "{}";
    String autoBaseProgress_ = "{}";
    String baseConfig_ = "{}";
    String serviceConfig_ = "{}";
    String receiverBaseReadback_ = "{\"state\":\"not_read\",\"chip\":\"Unicorecomm\",\"mode_kind\":\"unknown\"}";
    String surveyInResult_ = "{}";
    String baseStartupState_ = "pending";
    String baseStartupError_;
    bool baseStartupAttempted_ = false;
    String licenseToken_;
    String sensorType_ = "UNKNOWN";

    bool provisioned_ = false;
    StationLock stationLock_;
    bool licenseValid_ = false;
    bool mqttConnected_ = false;
    bool websocketConnected_ = false;
    bool controlPlaneFailOpen_ = false;
    volatile bool statusDirty_ = false;
    volatile bool workflowCancelRequested_ = false;

    uint32_t lastStatusMs_ = 0;
    uint32_t firmwareSize_ = 0;
    bool statusPublishConfirmed_ = false;
    uint32_t lastChipProbeMs_ = 0;
    uint32_t controlPlaneArmedMs_ = 0;
    uint32_t lastControlCheckMs_ = 0;
    uint32_t lastMqttAckMs_ = 0;
    uint32_t lastWebsocketAckMs_ = 0;
    uint32_t recoveryStartedMs_ = 0;
    int32_t lastMqttAckSequence_ = -1;
    int32_t lastWebsocketAckSequence_ = -1;
    int32_t statusSequence_ = 0;
    int32_t failOpenEntrySequence_ = -1;
    uint32_t receiverBaseReadbackMs_ = 0;
    uint32_t receiverBaseReadbackRevision_ = 0;
    uint32_t lastReceiverPollMs_ = 0;
    bool surveyInResultPending_ = false;
    bool surveyInReadbackReady_ = false;
    bool surveyInReadbackHasPosition_ = false;
    bool surveyInMonitoring_ = false;
    uint32_t surveyInStartedMs_ = 0;
    uint32_t surveyInTargetSeconds_ = 0;
    String statusBootId_;

    QueueHandle_t nmeaQueue_ = nullptr;
    QueueHandle_t rtcmQueue_ = nullptr;
    BackendAgentWebsocket *websocket_ = nullptr;
    TaskHandle_t workflowTask_ = nullptr;
    String workflowKind_;
    String workflowPayload_;

    void loadPreferences();
    bool saveStatePreferences();
    void ensureDefaultServiceConfig();
    String normalAgentState() const;
    void setupWebsocket();
    void refreshWebsocket();
    void loopWebsocket();
    void drainDataQueues();
    void evaluateControlPlaneFailover();
    bool startWorkflow(const String &kind, const String &payload);
    void runWorkflow();
    static void workflowTaskThunk(void *parameter);
    void requestWorkflowStop();
    bool writeGnssCommands(const GnssCmdBuilder::CommandList &commands);
    bool writeUnicoreCommand(const uint8_t *data, size_t length, String &error);
    void setWorkflowProgress(const String &phase, uint8_t step, uint8_t total,
                             const String &details = String());
    bool runAutoBaseWorkflow(JsonObject payload);
    bool runReferenceCheckWorkflow(JsonObject payload);
    bool restartConfiguredNtrip();
    int loopNtripRoverLocked();
    bool stopNtripRoverLocked();
    bool applyBaseCoordinates(double latitude, double longitude, double altitude);
    bool refreshReceiverReadback(String &error);
    void saveSurveyResult(JsonObjectConst bestPosition, const String &mode);
    void pollGnssStatus();
    bool transformToVn2000(double latitude, double longitude, double altitude,
                           double centralMeridianDeg, double scale,
                           double &localLatitude, double &localLongitude,
                           double &localAltitude, double &northing, double &easting);
    bool getProvinceProjection(const String &code, double &centralMeridianDeg,
                               double &scale, String &name) const;
    void markStatusDirty() { statusDirty_ = true; }

    String buildStatusPayload();
    String buildLwtPayload() const;
    String statusTopic() const;
    String rawDataTopic() const;
    String commandTopic() const;
    String controlAckTopic() const;

    void handleWebsocketText(const String &message);
    void handleCommand(const String &source, const String &message);
    void handleControlAck(const String &source, const String &message, bool retained);
    void recordCommandResult(const String &command, const String &source,
                             const String &status, const String &detail,
                             const String &commandId);
    void publishConfigState(const char *kind);
    void applyServiceStreamCommand(JsonObject payload);
    bool executeRawCommands(JsonArray commands, String &error, const String &sensorType = String());
    void publishTextFallback(const String &message);

    friend class BackendAgentWebsocket;
};

extern BackendAgent backendAgent;

// Expose device permissions and fail-open state to network transports.
bool backendDeviceLocked(uint8_t serverId = 0);
bool backendControlPlaneFailOpen();
bool backendDeviceProvisioned();
bool backendLicenseValid();
bool backendBaseStartupConfigured();

#endif // BACKEND_AGENT_H
