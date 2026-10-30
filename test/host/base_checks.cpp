#include "functions/Station_Lock.h"
#include "functions/Unicore_Output.h"
#include "functions/Base_Sample.h"
#include "functions/Device_Token.h"
#include "functions/gnss_cmd_builder.h"
#include "functions/Ubx_Reply.h"
#include "functions/Unicore_Reply.h"
#include "functions/Unicore_Identification.h"
#include "functions/Unicore_Log.h"
#include "functions/unicore_commands.h"
#include "functions/Ubx_Base_Status.h"
#include <cassert>
#include <iostream>
#include <string>

using namespace GnssCmdBuilder;

void checkPackets(const Command &bytes) {
    size_t offset = 0;
    while (offset < bytes.size()) {
        assert(bytes.size() - offset >= 8);
        assert(bytes[offset] == 0xB5 && bytes[offset + 1] == 0x62);
        const size_t size = bytes[offset + 4] | (size_t(bytes[offset + 5]) << 8);
        assert(offset + size + 8 <= bytes.size());
        uint8_t a = 0, b = 0;
        for (size_t i = offset + 2; i < offset + size + 6; ++i) { a += bytes[i]; b += a; }
        assert(bytes[offset + size + 6] == a && bytes[offset + size + 7] == b);
        offset += size + 8;
    }
}

int main(int argc, char **argv) {
    uint8_t svin[40] = {};
    const auto put32 = [](uint8_t *p, uint32_t value) {
        for (int i = 0; i < 4; ++i) p[i] = value >> (8*i);
    };
    put32(svin+8, 120); put32(svin+28, 50000); put32(svin+32, 95);
    put32(svin+12, uint32_t(-161974355)); put32(svin+16, 572912270); put32(svin+20, 228017820);
    svin[24] = uint8_t(-75); svin[25] = 7; svin[26] = 74;
    UbxBaseStatus baseStatus; baseStatus.mode = 1;
    svin[37] = 1;
    assert(baseStatus.svin(svin,40) && baseStatus.active && baseStatus.hasPosition);
    assert(baseStatus.duration == 120 && baseStatus.observations == 95 && baseStatus.accuracy == 5);
    svin[36] = 1; // Even a valid position is not final while the survey is active.
    assert(baseStatus.svin(svin,40) && baseStatus.hasPosition && baseStatus.active);
    svin[37] = 0;
    assert(baseStatus.svin(svin,40) && baseStatus.valid && baseStatus.hasPosition);
    assert(std::abs(baseStatus.latitude-21.08498763643) < 1e-10);
    assert(std::abs(baseStatus.longitude-105.7867141884) < 1e-10);
    assert(std::abs(baseStatus.height-2.9192763) < 1e-6);
    svin[24] = 100;
    assert(!baseStatus.svin(svin,40));
    assert(!baseStatus.svin(svin,39));
    uint8_t tmode[40] = {}; tmode[2] = 2; tmode[3] = 1;
    put32(tmode+4, 210849876); put32(tmode+8, 1057867141); put32(tmode+12, 291);
    tmode[16] = 12; tmode[17] = 88; tmode[18] = 94;
    assert(baseStatus.tmode(tmode,40) && baseStatus.mode == 2 && baseStatus.hasPosition);
    assert(std::abs(baseStatus.latitude-21.084987612) < 1e-11);
    assert(std::abs(baseStatus.longitude-105.786714188) < 1e-11);
    assert(std::abs(baseStatus.height-2.9194) < 1e-10);
    assert(!baseStatus.ecef(0,0,0));
    assert(baseStatus.ecef(6378137,0,0) && std::abs(baseStatus.height) < 1e-8);
    const std::string modeLog = "#MODE,81,GPS,FINE,2230,547967000,0,0,18,518;MODE ROVER SURVEY,*1B";
    assert(validUnicoreLogChecksum(modeLog.data(), modeLog.size()));
    const std::string crcLog = "#BASEINFOA,56,GPS,FINE,2190,376748000,0,0,18,153;00000000,-2160493.199,4383620.763,4084734.120,\"0000\",0*2edbd87a";
    assert(validUnicoreLogChecksum(crcLog.data(), crcLog.size()));
    auto corruptLog = crcLog;
    corruptLog[20] ^= 1;
    assert(!validUnicoreLogChecksum(corruptLog.data(), corruptLog.size()));
    assert(!validUnicoreLogChecksum(modeLog.data(), modeLog.size() - 1));
    assert(!validUnicoreLogChecksum("#MODE;MODE BASE", 15));
    assert(!validUnicoreLogChecksum(nullptr, 0));
    UnicoreIdentification identity;
    const auto feedIdentity = [&](const std::string &bytes) {
        bool found = false;
        for (unsigned char byte : bytes) found = identity.feed(byte) || found;
        return found;
    };
    assert(!feedIdentity(std::string("\xD3\0\x17#noise\0", 11)));
    assert(!feedIdentity("$command,versiona,response: OK\r\n"));
    assert(!feedIdentity("#MODE,97,GPS;\"UM980\"\r\n"));
    assert(!feedIdentity("#VERSIONA,97,GPS;\"OTHER\",\"VERSION\"*00\r\n"));
    assert(!feedIdentity("#VERSIONA,97,GPS;\"UM98"));
    assert(feedIdentity("0\",\"R4.10\"*12345678\r\n"));
    assert(!feedIdentity("#" + std::string(600, 'x') + "\r\n"));
    assert(feedIdentity("#versiona,97,GPS;\"UM982\"*12345678\r\n"));
    const std::string umCommand = "mode rover\r\n";
    const auto umReply = [&]() {
        return UnicoreReply(reinterpret_cast<const uint8_t *>(umCommand.data()), umCommand.size());
    };
    const auto feedUm = [](UnicoreReply &reply, const std::string &text) {
        size_t matched = 0;
        for (unsigned char c : text) matched += reply.feed(c);
        return matched;
    };
    auto um = umReply();
    assert(feedUm(um, "$GPGGA,noise\r\n$command,saveconfig,response: OK\r\n") == 0);
    assert(um.state == UnicoreReply::State::Pending);
    assert(feedUm(um, "$command,MODE ROVER,response: OK\r\n") == 1);
    assert(um.state == UnicoreReply::State::Ok);
    for (const std::string response : {"NOT OK", "PARSING FAILD NO MATCHING FUNC", "ERROR"}) {
        auto rejected = umReply();
        assert(feedUm(rejected, "$command,mode rover,response: " + response + "\r\n") == 1);
        assert(rejected.state == UnicoreReply::State::Error);
    }
    um = umReply();
    assert(feedUm(um, "$command,mode rover,response: \r\n") == 0);
    assert(feedUm(um, "$command,mode rover,response: OK*00\r\n") == 0);
    assert(feedUm(um, "$command,mode rover,response: OK*xx\r\n") == 0);
    // Real N4 response example; checksum includes '$', unlike standard NMEA.
    const std::string survey = "MODE ROVER SURVEY\r\n";
    UnicoreReply checksummed(reinterpret_cast<const uint8_t *>(survey.data()), survey.size());
    assert(feedUm(checksummed, "$command,MODE ROVER SURVEY,response: OK*1F\r\n") == 1);
    assert(checksummed.state == UnicoreReply::State::Ok);
    um = umReply();
    assert(feedUm(um, "$command," + std::string(500, 'x') + "\r\n") == 0);
    assert(feedUm(um, "$command,broken$command,mode rover,response: ok\r\n") == 1);
    assert(um.state == UnicoreReply::State::Ok);
    for (const std::string &invalid : std::vector<std::string>{"", "mode rover\r\nsaveconfig\r\n", std::string("mode \xB5\x62", 7)}) {
        UnicoreReply rejected(reinterpret_cast<const uint8_t *>(invalid.data()), invalid.size());
        assert(!rejected.key[0]);
    }
    assert(!UnicoreReply(nullptr, 0).key[0]);
    assert(buildRoverCommand(GnssSensorType::Invalid).empty());
    assert(sensorTypeFromString("Unicorecomm") == GnssSensorType::Unicore);
    const auto tokenValid = [](const std::string &token) {
        return isDeviceBearerToken(token.data(), token.size());
    };
    assert(tokenValid("dt_" + std::string(40, 'a')));
    assert(tokenValid("dt_" + std::string(64, 'Z')));
    assert(tokenValid("dt_" + std::string(38, 'a') + "_-"));
    assert(!tokenValid("dt_" + std::string(39, 'a')));
    assert(!tokenValid("dt_" + std::string(65, 'a')));
    assert(!tokenValid("DT_" + std::string(40, 'a')));
    assert(!tokenValid("dt_" + std::string(39, 'a') + "!"));
    assert(!tokenValid("dt_" + std::string(39, 'a') + "\n"));
    assert(!isDeviceBearerToken(nullptr, 43));
    assert(freshRtkSample(5000, 4000, "RTK_FIXED"));
    assert(!freshRtkSample(5000, 4000, "RTK_FLOAT"));
    assert(!freshRtkSample(5000, 0, "RTK_FIXED"));
    assert(!freshRtkSample(5000, 1000, "RTK_FIXED"));
    assert(!freshRtkSample(5000, 4000, "GPS_FIX"));
    assert(!freshRtkSample(5000, 4000, "BASE_FIXED"));
    assert(freshRtkSample(100, UINT32_MAX - 100, "RTK_FIXED"));
    StationLock lock;
    const int second[] = {2};
    assert(lock.setScoped(second, 1));
    assert(!lock.locked() && !lock.locked(1) && lock.locked(2));
    StationLock rebooted;
    rebooted.restore(lock.all, lock.servers);
    assert(!rebooted.locked(1) && rebooted.locked(2));
    const int invalid[] = {3};
    assert(!lock.setScoped(invalid, 1) && lock.locked(2));
    const int both[] = {1, 2};
    assert(lock.setScoped(both, 2) && !lock.locked() && lock.locked(1) && lock.locked(2));
    lock.restore(true, 0);
    assert(lock.locked() && lock.locked(1) && lock.locked(2));
    assert(lock.setScoped(nullptr, 0) && !lock.locked(1) && !lock.locked(2));
    lock.restore(false, 255);
    assert(lock.locked() && lock.locked(1) && lock.locked(2));

    std::string ascii = "rtcm1074 com3 1\r\n";
    routeUnicoreOutput(reinterpret_cast<uint8_t *>(&ascii[0]), ascii.size(), 2);
    assert(ascii == "rtcm1074 com2 1\r\n");
    for (const auto &message : {"rtcm1006", "rtcm1077", "gpgga", "gpgst"}) {
        for (const auto &port : {"COM1", "com2", "CoM3"}) {
            std::string current = std::string(message) + " " + port + " 1\r\n";
            const size_t length = routeUnicoreOutput(reinterpret_cast<uint8_t *>(current.data()), current.size(), 0);
            current.resize(length);
            assert(current == std::string(message) + " 1\r\n");
        }
    }
    std::string other = "config com3 115200\r\n";
    routeUnicoreOutput(reinterpret_cast<uint8_t *>(&other[0]), other.size(), 2);
    assert(other == "config com3 115200\r\n");
    assert(routeUnicoreOutput(reinterpret_cast<uint8_t *>(other.data()), other.size(), 0) == other.size());
    assert(other == "config com3 115200\r\n");
    for (size_t length = 0; length < ascii.size(); ++length) {
        auto shortened = ascii.substr(0, length);
        routeUnicoreOutput(reinterpret_cast<uint8_t *>(shortened.data()), length, 2);
        routeUnicoreOutput(reinterpret_cast<uint8_t *>(shortened.data()), length, 0);
    }
    GnssOptions usb, uart;
    const auto ephemeris = UnicoreCommands::buildEphemerisOutputCommands("com2");
    assert(ephemeris.size() == 5);
    assert(UnicoreCommands::buildEphemerisOutputCommands("COM9").empty());
    const auto unicoreAll = UnicoreCommands::buildBaseAllRtcmOutputCommand("COM2", true);
    assert(unicoreAll.size() == 18);
    const auto unicoreAllText = commandListToBytes(unicoreAll);
    const std::string unicoreAllOutput(unicoreAllText.begin(), unicoreAllText.end());
    assert(unicoreAllOutput.find("rtcm1006 COM2 1\r\n") != std::string::npos);
    assert(unicoreAllOutput.find("rtcm1044 COM2 1\r\n") != std::string::npos);
    assert(UnicoreCommands::buildBaseAllRtcmOutputCommand("COM9", true).empty());
    const auto currentOutput = commandListToBytes(UnicoreCommands::buildBaseAllRtcmOutputCommand("CURRENT", true));
    const std::string currentOutputText(currentOutput.begin(), currentOutput.end());
    assert(currentOutputText.find("rtcm1006 1\r\n") != std::string::npos);
    assert(currentOutputText.find("rtcm1044 1\r\n") != std::string::npos);
    assert(currentOutputText.find("CURRENT") == std::string::npos);
    assert(currentOutputText.find("COM") == std::string::npos);
    const auto singleCurrent = commandListToBytes(UnicoreCommands::buildBaseRtcmOutputCommand("1006", "CURRENT", true));
    assert(std::string(singleCurrent.begin(), singleCurrent.end()) == "rtcm1006 1\r\nsaveconfig\r\n");
    for (const auto &command : ephemeris) {
        UnicoreReply reply(command.data(), command.size());
        assert(reply.key[0]);
        const std::string line(command.begin(), command.end());
        assert(line.find(" COM2 1\r\n") != std::string::npos);
    }
    uart.ports = {"UART1"};
    uart.outputMode = OutputMode::RtcmOnly;
    for (const auto type : {GnssSensorType::Ublox, GnssSensorType::Unicore}) {
        const auto survey = buildBaseSurveyInCommand(300, 1.0, type, uart);
        const auto fixed = buildBaseFixedLlaCommand(10.75, 106.5, 12.5, 0.01, type, uart);
        const auto rover = buildRoverCommand(type);
        assert(!survey.empty() && !fixed.empty() && !rover.empty());
        if (type == GnssSensorType::Ublox) {
            const auto all = buildBaseAllRtcmOutputCommand("UART1", true, type);
            assert(all.size() == 2);
            assert(survey[0][8] == 1 && fixed[0][8] == 2 && fixed[0][9] == 1);
            assert(survey[0][30] == 44 && survey[0][31] == 1);
            assert(rover[1][8] == 0);
            for (const size_t packet : {8U, 9U}) {
                // GGA/GST must reach a receiver wired to UART1, UART2 or USB.
                assert(rover[packet][9] == 1 && rover[packet][10] == 1 && rover[packet][11] == 1);
            }
            for (const uint32_t key : {0x10730004U, 0x10750004U, 0x10770004U,
                                       0x10740002U, 0x10760002U, 0x10780002U,
                                       0x209100BBU, 0x209100BCU, 0x209100BDU}) {
                bool enabled = false;
                for (const auto &settings : rover) {
                  if (settings[2] != 6 || settings[3] != 0x8A) continue;
                  for (size_t i = 10; i + 5 <= settings.size() - 2; i += 5) {
                    const uint32_t actual = uint32_t(settings[i]) | uint32_t(settings[i + 1]) << 8
                        | uint32_t(settings[i + 2]) << 16 | uint32_t(settings[i + 3]) << 24;
                    if (actual == key) enabled = settings[i + 4] == 1;
                  }
                }
                assert(enabled);
            }
            // No optional UART2 key may share the TMODE configuration packet.
            assert(rover[0].size() == 17 && rover[0][14] == 0);
            assert(rover[0][10] == 0x01 && rover[0][11] == 0x00
                   && rover[0][12] == 0x03 && rover[0][13] == 0x20);
            UbxReply reply;
            for (uint8_t noise : {0x00, 0xB5, 0xB5, 0x00}) assert(!reply.feed(noise));
            size_t replies = 0;
            for (uint8_t byte : rover[1]) replies += reply.feed(byte);
            assert(replies == 1 && reply.frame[8] == 0 && reply.length == 48);
            auto corrupt = rover[1];
            corrupt.back() ^= 1;
            for (uint8_t byte : corrupt) assert(!reply.feed(byte));
            // Oversized UBX lengths must be discarded without overrunning.
            for (uint8_t byte : {0xB5, 0x62, 0x06, 0x71, 0xFF, 0xFF}) assert(!reply.feed(byte));
            replies = 0;
            for (uint8_t byte : rover[1]) replies += reply.feed(byte);
            assert(replies == 1);
            for (const auto &commands : {survey, fixed, rover}) for (const auto &packet : commands) checkPackets(packet);
        } else {
            const auto text = commandListToBytes(fixed);
            const std::string output(text.begin(), text.end());
            assert(output.find("mode base 10.7500000000 106.5000000000 12.5000\r\n") != std::string::npos);
            assert(output.find("com3") == std::string::npos);
            assert(output.find("gpgga 1\r\n") == std::string::npos);
            assert(output.find("gpgsa 1\r\n") == std::string::npos);
            assert(output.find("gpgsv 1\r\n") == std::string::npos);
            assert(output.find("gpgst 1\r\n") == std::string::npos);
            assert(output.find("rtcm1074 1") != std::string::npos);
            assert(output.find("com2") == std::string::npos);
            for (const auto &commands : {survey, fixed}) {
                const auto bytes = commandListToBytes(commands);
                const std::string configured(bytes.begin(), bytes.end());
                assert(configured.find("gpgga 1\r\n") == std::string::npos);
                assert(configured.find("gpgsa 1\r\n") == std::string::npos);
                assert(configured.find("gpgsv 1\r\n") == std::string::npos);
                assert(configured.find("gpgst 1\r\n") == std::string::npos);
                for (const char *id : {"1019", "1020", "1042", "1045"})
                    assert(configured.find(std::string("rtcm") + id + " 1\r\n") != std::string::npos);
            }
        }
    }
    GnssOptions diagnostics = uart;
    diagnostics.outputMode = OutputMode::DiagnosticsRtcm;
    const auto diagnosticSurvey = buildBaseSurveyInCommand(
        120, 10.0, GnssSensorType::Unicore, diagnostics);
    const auto diagnosticText = commandListToBytes(diagnosticSurvey);
    const std::string diagnosticOutput(diagnosticText.begin(), diagnosticText.end());
    assert(diagnosticOutput.find("config undulation 0.0\r\n") < diagnosticOutput.find("mode base time "));
    assert(diagnosticOutput.find("gpgga 1\r\n") != std::string::npos);
    assert(diagnosticOutput.find("rtcm1074 1\r\n") != std::string::npos);

    for (const auto mode : {OutputMode::DiagnosticsRtcm, OutputMode::RtcmOnly}) {
        usb.outputMode = mode;
        uart.outputMode = mode;
        auto output = buildUbloxOutputConfigCommand(usb);
        routeUbloxOutput(output.data(), output.size());
        checkPackets(output);
        assert(output == buildUbloxOutputConfigCommand(uart));
    }

    // Route hexadecimal packets from stdin and emit the resulting bytes for comparison.
    if (argc > 1) {
        const bool ublox = std::string(argv[1]) == "Ublox";
        std::string hex;
        while (std::cin >> hex) {
            Command packet;
            for (size_t i = 0; i < hex.size(); i += 2) packet.push_back(std::stoul(hex.substr(i, 2), nullptr, 16));
            if (ublox) {
                routeUbloxOutput(packet.data(), packet.size());
                checkPackets(packet);
            } else {
                packet.resize(routeUnicoreOutput(packet.data(), packet.size(), 0));
            }
            for (uint8_t byte : packet) printf("%02x", byte);
            printf("\n");
        }
    } else {
        std::cout << "PASS: both receiver command sets, checksums, UART routing, scoped/persistent locks\n";
    }
}
