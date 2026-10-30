#pragma once
#include <Arduino.h>
#include <Client.h>
#include <vector>

class BackendAgent;

// RFC6455 client shared by Wi-Fi and cellular TCP transports.
class BackendAgentWebsocket {
public:
    explicit BackendAgentWebsocket(BackendAgent *owner);

    void begin(const String &host, uint16_t port, const String &path, Client &client);

    void loop();

    bool connected() const;

    void disconnect(bool permanent);

    bool sendText(const String &text);

private:
    BackendAgent *owner_ = nullptr;
    Client *client_ = nullptr;
    String host_;
    String path_;
    uint16_t port_ = 0;
    uint32_t nextAttemptMs_ = 0;
    uint32_t lastPingMs_ = 0;
    bool connected_ = false;
    std::vector<uint8_t> rx_;

    void connectNow();

    void parseFrames();

    void sendControl(uint8_t opcode, const uint8_t *payload, size_t length);
};
