#ifndef BACKEND_AGENT_H
#define BACKEND_AGENT_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include "functions/backend/Station_Lock.h"
#include "functions/gnss/gnss_cmd_builder.h"
#include "functions/gnss/Ubx_Base_Status.h"
#include "functions/gnss/Unicore_Reply.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <vector>

class BackendAgentWebsocket;
struct GnssTelemetrySnapshot;

// Device agent for backend commands, status, provisioning and base workflows.
// Receiver coordinates come from NMEA or Unicore readback; RTCM is forwarded.
class BackendAgent
{
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
    bool baseStartupConfigured() const
    {
        return baseStartupState_ == "fixed_restored" || baseStartupState_ == "survey_started" || baseStartupState_ == "preserved" || baseStartupState_ == "configured";
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
    void saveDetectedChip(const char *chipType);
    bool probeUnicore(Stream &stream, String &identityLine);
    bool probeUblox(Stream &stream);
    bool probePassiveUblox(Stream &stream);
    void ensureDefaultServiceConfig();
    String normalAgentState() const;
    void setupWebsocket();
    void refreshWebsocket();
    void loopWebsocket();
    void drainDataQueues();
    inline void retryUnknownChipProbe();
    void evaluateControlPlaneFailover();
    bool startWorkflow(const String &kind, const String &payload);
    void runWorkflow();
    static void workflowTaskThunk(void *parameter);
    void requestWorkflowStop();
    bool writeGnssCommands(const GnssCmdBuilder::CommandList &commands);
    bool writeUnicoreCommand(const uint8_t *data, size_t length, String &error);
    bool waitForUnicoreReply(UnicoreReply &reply, bool written);
    void handleUnicoreAck(const UnicoreReply &reply, const uint8_t *data, size_t length);
    bool exchangeUbloxRoverCommand(const GnssCmdBuilder::Command &command, bool poll);
    bool configureUbloxRover();
    bool waitForRoverGga(uint32_t startedMs);
    void setWorkflowProgress(const String &phase, uint8_t step, uint8_t total,
                             const String &details = String());
    bool runAutoBaseWorkflow(JsonObject payload);
    bool prepareAutoBaseConfig(JsonObject payload, String &host, uint16_t &port,
                               String &username, String &password, String &mountpoint,
                               uint8_t &ntripVersion, uint32_t &timeoutSeconds,
                               uint8_t &fixedStreakRequired);
    bool startAutoBaseRover(const String &host, uint16_t port, const String &username,
                            const String &password, const String &mountpoint,
                            uint8_t ntripVersion);
    bool waitForAutoBaseFix(uint32_t timeoutSeconds, uint8_t fixedStreakRequired,
                            bool &cancelled, String &failureReason);
    bool collectAutoBaseSamples(int configuredSamples, std::vector<double> &latitudes,
                                std::vector<double> &longitudes,
                                std::vector<double> &altitudes, bool &cancelled,
                                String &failureReason);
    bool resolveAutoBaseProjection(JsonObject payload, double rawLongitude,
                                   double &centralMeridianDeg, double &scale,
                                   String &provinceName, String &provinceCode,
                                   String &failureReason);
    bool completeAutoBaseSetup(JsonObject payload, const std::vector<double> &latitudes,
                               const std::vector<double> &longitudes,
                               const std::vector<double> &altitudes,
                               String &resultJson, String &failureReason);
    String buildAutoBaseResult(double finalLatitude, double finalLongitude,
                               double finalAltitude, double rawLatitude,
                               double rawLongitude, double rawAltitude,
                               bool transformed, bool transportRestored,
                               size_t sampleCount, const String &projectionSource,
                               const String &provinceCode, const String &provinceName,
                               double centralMeridianDeg, double scale);
    bool runReferenceCheckWorkflow(JsonObject payload);
    bool prepareReferenceCheckConfig(JsonObject payload, JsonObject &reference,
                                     double &referenceLatitude, double &referenceLongitude,
                                     double &referenceAltitude, double &restoreLatitude,
                                     double &restoreLongitude, double &restoreAltitude,
                                     String &host, uint16_t &port, String &username,
                                     String &password, String &mountpoint,
                                     uint32_t &timeoutSeconds, uint16_t &requestedSamples);
    inline bool startReferenceRover(const String &host, uint16_t port, const String &username,
                                    const String &password, const String &mountpoint);
    bool waitForReferenceFix(uint32_t timeoutSeconds, String &failureReason);
    bool collectReferenceSamples(uint16_t requestedSamples, std::vector<double> &latitudes,
                                 std::vector<double> &longitudes,
                                 std::vector<double> &altitudes, String &failureReason);
    String buildReferenceCheckResult(JsonObject reference, double measuredLatitude,
                                     double measuredLongitude, double measuredAltitude,
                                     size_t sampleCount);
    bool restartConfiguredNtrip();
    int loopNtripRoverLocked();
    bool stopNtripRoverLocked();
    bool applyBaseCoordinates(double latitude, double longitude, double altitude);
    bool refreshReceiverReadback(String &error);
    bool refreshUbloxReadback(String &error);
    void updateUbloxSurveyState(const UbxBaseStatus &status);
    void logUbloxReadback(const UbxBaseStatus &status);
    void appendUbloxPosition(DynamicJsonDocument &doc, const UbxBaseStatus &status,
                             const String &mode);
    bool refreshUnicoreReadback(String &error);
    bool prepareStartupConfig(DynamicJsonDocument &config, JsonObject &base,
                              bool &saved, String &method, String &error);
    bool readStartupReceiverMode(GnssCmdBuilder::GnssSensorType detected, String &error);
    bool buildStartupCommands(JsonObject base, GnssCmdBuilder::GnssSensorType detected,
                              bool restoreFixed, GnssCmdBuilder::CommandList &commands,
                              double &fixedLatitude, double &fixedLongitude,
                              double &fixedAltitude, bool &ellipsoid, String &error);
    bool verifyStartupReadback(GnssCmdBuilder::GnssSensorType detected, bool restoreFixed,
                               double fixedLatitude, double fixedLongitude,
                               double fixedAltitude, bool ellipsoid, String &error);
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
    String statusForPayload(bool anyStream, bool onDemand) const;
    void appendStatusFields(JsonDocument &doc, const String &status,
                            const GnssTelemetrySnapshot &gnss, uint32_t nowMs,
                            const String &timestamp) const;
    void appendAutoBaseProgress(JsonDocument &doc, const GnssTelemetrySnapshot &gnss,
                                uint32_t nowMs) const;
    void appendSystemInfo(JsonDocument &doc, uint32_t nowMs,
                          const String &timestamp) const;
    void appendStartupInfo(JsonObject system) const;
    JsonObject appendSurveyInfo(JsonObject system, uint32_t nowMs) const;
    void appendReceiverReadback(JsonObject system, JsonObject survey) const;
    void appendResourceInfo(JsonObject system, const String &timestamp) const;
    void appendGnssStatus(JsonDocument &doc, const GnssTelemetrySnapshot &gnss,
                          uint32_t nowMs) const;
    void appendNtripStatus(JsonDocument &doc, const JsonObjectConst &service,
                           bool server1Active, bool server2Active) const;
    void appendTransportStatus(JsonDocument &doc, const GnssTelemetrySnapshot &gnss,
                               uint32_t nowMs, int32_t signal) const;
    String statusTopic() const;
    String rawDataTopic() const;
    String commandTopic() const;
    String controlAckTopic() const;

    void handleWebsocketText(const String &message);
    void handleCommand(const String &source, const String &message);
    void executeCommand(const String &source, const String &command,
                        const String &commandId, JsonDocument &doc,
                        JsonObject payload);
    bool executeLicenseCommand(JsonObject payload, String &error, String &detail,
                               bool &restartAfterCommand);
    bool executeServiceConfigCommand(JsonObject payload, String &error, String &detail);
    bool executeRawCommand(JsonDocument &doc, JsonObject payload,
                           String &error, String &detail);
    bool executeAutoBaseCommand(JsonObject payload, String &error, String &detail);
    bool executeReferenceCheckCommand(JsonObject payload, String &error, String &detail);
    bool executeProvinceCommand(JsonObject payload, String &error, String &detail);
    bool executeBasicCommand(const String &command, JsonObject payload,
                             String &error, String &detail);
    bool executeStreamCommand(JsonObject payload, String &error, String &detail);
    bool executeBaseStatusCommand(String &error, String &detail);
    bool executeStopWorkflowCommand(String &detail);
    void finishCommand(const String &source, const String &command,
                       const String &commandId, bool success,
                       const String &error, const String &detail,
                       bool restartAfterCommand);
    void handleControlAck(const String &source, const String &message, bool retained);
    bool applyLockCommand(JsonObjectConst payload, String &error);
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
