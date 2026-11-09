"""Exercise complete agent GNSS verification, polling, UART ownership and Rover transition."""
from pathlib import Path
import re
import tempfile
from host_coverage import compile_cpp, run_cpp, mapped

root=Path(__file__).resolve().parents[2]
source_path=root/'src/functions/backend/Backend_Agent_Gnss.cpp'
source=re.sub(r'^#include.*$','',source_path.read_text(),flags=re.M)
header=re.sub(r'^#include.*$','',(root/'include/functions/backend/Backend_Agent.h').read_text(),flags=re.M).replace('private:','public:')
harness=r'''
#include <Arduino.h>
#include <ArduinoJson.h>
#include "functions/backend/Station_Lock.h"
#include "functions/backend/Backend_Agent_Codec.h"
#include "functions/gnss/gnss_cmd_builder.h"
#include "functions/gnss/Ubx_Navigation.h"
#include "functions/gnss/Ubx_Base_Status.h"
#include "functions/gnss/Ubx_Config_Verification.h"
#include "functions/gnss/Unicore_Config_Verification.h"
#include "functions/gnss/Ubx_Reply.h"
#include "functions/gnss/Unicore_Reply.h"
#include <cassert>
#include <cstring>
#include <array>
constexpr unsigned VERIFICATION_DOCUMENT_SIZE=12288,MUTEX_TIMEOUT_MS=1500,GNSS_UNICORE_PORT=0;
using QueueHandle_t=void*;using TaskHandle_t=void*;
class Stream;
String operator+(const String &text,uint8_t value){return String(std::string(text)+std::to_string(value));}
bool readOk=true,positionOk=true,mismatch=false,freshGga=true;
unsigned saves=0,reconciled=0;
void delay(unsigned ms){hostMillis+=ms;}
struct GnssTelemetrySnapshot{uint32_t lastGgaMs=0;std::array<char,128> lastGga={'$','G','G','A'};};
GnssTelemetrySnapshot getGnssTelemetrySnapshot(){GnssTelemetrySnapshot value;value.lastGgaMs=freshGga?millis():0;return value;}
namespace BackendAgentInternal{String epochString(){return "123";}}
UbxBaseStatus baseStatus;uint8_t observedMode[40]={};
namespace UbloxReadback{
bool readTmode(uint8_t *out,String &error){std::memcpy(out,observedMode,40);if(!readOk)error="unread";return readOk;}
bool readOutputValues(const uint8_t *records,size_t length,uint8_t *out,String&){for(size_t i=0;i<length/5;++i)out[i]=records[i*5+4]^(mismatch?1:0);return readOk;}
bool readBaud(uint8_t,uint32_t &baud,String&){baud=115200;return readOk;}
bool readBaseStatus(UbxBaseStatus &out,String &error){out=baseStatus;if(!readOk)error="unread";return readOk;}
bool readNavigation(UbxNavigation &out,String&){out.valid=true;out.haccMm=12;return readOk;}
bool readDop(UbxDop &out,String&){out.hdop=80;return readOk;}
bool readSatellites(String &out,String&){out="satellite payload";return readOk;}
bool readVersion(String &out,String &error){out="version";if(!readOk)error="version unread";return readOk;}
}
namespace UnicoreReadback{
String mode="MODE BASE 21 105 12",logs="#UNILOGLIST,header;< 2\r\n< GPGGA COM2 1\r\n< RTCM1074 COM2 1\r\n\r\n";
bool readMode(String &out,String &error){out=mode;if(!readOk)error="mode unread";return readOk;}
const char *modeKind(const String &text){return text.startsWith("MODE BASE TIME")?"survey_in":text.startsWith("MODE BASE")?"base":"rover";}
bool readLogList(String &out,String&){out=logs;return readOk;}
bool readBestPosition(JsonObject out,String &error){if(!positionOk){error="unread";return false;}out["position_type"]="FIXEDPOS";out["latitude"]=21;out["longitude"]=105;out["height_msl"]=12;out["horizontal_sigma_m"]=0.015;return true;}
}
'''
stubs=r'''
bool BackendAgent::saveStatePreferences(){++saves;return true;}
void BackendAgent::reconcileBaseStartupReadback(JsonObjectConst){++reconciled;}
'''
checks=r'''
std::vector<uint8_t> ubx(uint8_t cls,uint8_t id,std::vector<uint8_t> payload){
    std::vector<uint8_t> bytes{0xb5,0x62,cls,id,uint8_t(payload.size()),uint8_t(payload.size()>>8)};
    bytes.insert(bytes.end(),payload.begin(),payload.end());uint8_t a=0,b=0;
    for(size_t i=2;i<bytes.size();++i){a+=bytes[i];b+=a;}bytes.push_back(a);bytes.push_back(b);return bytes;
}
void ascii(const char *name,bool ok=true){std::string text=std::string("$command,")+name+",response: "+(ok?"OK":"ERROR")+"\r\n";Serial1.replies.emplace_back(text.begin(),text.end());}
String state(const String &serialized){DynamicJsonDocument result(12288);assert(deserializeJson(result,serialized)==DeserializationError::Ok);return result["state"]|String();}
void resetUart(){assert(acquiredLocks==releasedLocks);Serial1={};deniedLock=nullptr;rtcmBufferMutex=reinterpret_cast<void*>(1);gnssSerialMutex=reinterpret_cast<void*>(2);hostMillis=1000;}
void expectation(DynamicJsonDocument &doc,bool catalogue){
    using namespace BackendAgentInternal;
    uint8_t tmode[40]={};tmode[2]=2;tmode[3]=1;std::memcpy(observedMode,tmode,40);
    doc.clear();doc["tmode_b64"]=encodeBase64(tmode,40);
    auto records=[](unsigned port){
        const uint32_t keys[]={UbxConfigVerification::enabledKey(port),0x20910001U,0x209102BDU};
        std::vector<uint8_t> result;for(uint32_t key:keys){for(unsigned i=0;i<4;++i)result.push_back(key>>(8*i));result.push_back(1);}return result;
    };
    auto expected=records(1);doc["output_b64"]=encodeBase64(expected.data(),expected.size());
    if(catalogue){
        JsonArray fields=doc["readback"].createNestedArray("fields");for(const char *name:{"enabled","diagnostics","rtcm"})fields.add(name);
        JsonArray ports=doc["readback"].createNestedArray("ports");unsigned index=0;
        for(const char *name:{"I2C","UART1","UART2","USB","SPI"}){auto port=ports.createNestedObject();port["name"]=name;auto bytes=records(index++);port["output_b64"]=encodeBase64(bytes.data(),bytes.size());}
    }
}
int main(){
    BackendAgent agent;String error;DynamicJsonDocument payload(12288);
    for(bool catalogue:{false,true})for(unsigned outcome=0;outcome<3;++outcome){
        expectation(payload,catalogue);readOk=outcome!=2;mismatch=outcome==1;error.clear();
        assert(agent.verifyGnssConfig(payload.as<JsonObjectConst>(),"verify-1",error));
        assert(state(agent.gnssVerification_)==(outcome==2?"unread":outcome==1?"mismatch":"match"));
    }
    readOk=true;mismatch=false;
    for(unsigned invalid=0;invalid<5;++invalid){
        expectation(payload,true);
        if(invalid==0)payload["tmode_b64"]="bad";
        if(invalid==1)payload["readback"]["fields"].as<JsonArray>().clear();
        if(invalid==2)payload["readback"]["ports"][0]["name"]="bad";
        if(invalid==3)payload["readback"]["ports"][0]["output_b64"]="bad";
        if(invalid==4)payload["readback"]["ports"][1]["name"]="I2C";
        error.clear();assert(!agent.verifyGnssConfig(payload.as<JsonObjectConst>(),"bad",error)&&!error.isEmpty());
    }
    payload.clear();deserializeJson(payload,R"({"coords":{"lat":21,"lon":105,"alt":12},"logs":["GPGGA","RTCM1074"],"port":"COM2","altitude_reference":"ELLIPSOID"})");
    assert(agent.verifyUnicoreConfig(payload.as<JsonObjectConst>(),"um",error)&&state(agent.gnssVerification_)=="match");
    for(const char *mode:{"MODE ROVER","MODE BASE 22 105 12","MODE BASE"}){UnicoreReadback::mode=mode;assert(agent.verifyUnicoreConfig(payload.as<JsonObjectConst>(),"um",error));assert(state(agent.gnssVerification_)!="match");}
    UnicoreReadback::mode="MODE BASE 21 105 12";
    for(unsigned mode=0;mode<3;++mode){
        resetUart();readOk=true;agent.sensorType_="Ublox";baseStatus={};baseStatus.mode=mode;baseStatus.hasPosition=true;baseStatus.valid=true;baseStatus.latitude=21;baseStatus.longitude=105;baseStatus.height=12;baseStatus.targetSeconds=120;
        agent.surveyInResultPending_=true;assert(agent.refreshReceiverReadback(error));
        assert(agent.receiverBaseReadbackRevision_>=1&&agent.surveyInReadbackReady_&&agent.statusDirty_);
    }
    auto before=saves;agent.saveSurveyResult(payload.as<JsonObjectConst>(),"ignored");assert(saves==before);
    readOk=false;assert(!agent.refreshReceiverReadback(error));readOk=true;
    agent.sensorType_="Unicorecomm";UnicoreReadback::mode="MODE BASE TIME 120";agent.surveyInMonitoring_=false;agent.surveyInResultPending_=true;
    assert(agent.refreshReceiverReadback(error)&&agent.surveyInTargetSeconds_==120&&agent.unicoreQualityMs_>0);
    positionOk=false;assert(agent.refreshReceiverReadback(error)&&!agent.surveyInReadbackHasPosition_);positionOk=true;
    readOk=false;assert(!agent.refreshReceiverReadback(error));readOk=true;
    UnicoreReadback::mode="MODE ROVER";assert(agent.refreshReceiverReadback(error)&&!agent.surveyInMonitoring_);
    agent.sensorType_="Ublox";agent.lastReceiverPollMs_=0;hostMillis=5000;agent.pollGnssStatus();assert(agent.navigation_.valid&&agent.dop_.hdop==80&&agent.satelliteReadbackMs_==5000);
    readOk=false;hostMillis+=30000;agent.pollGnssStatus();assert(agent.versionReadbackError_=="version unread");readOk=true;
    agent.stationLock_.all=true;before=reconciled;hostMillis+=30000;agent.pollGnssStatus();assert(reconciled==before);agent.stationLock_.all=false;
    agent.sensorType_="UNKNOWN";agent.pollGnssStatus();assert(!agent.applyRoverMode());
    for(unsigned failure=0;failure<6;++failure){
        resetUart();error.clear();if(failure==0)rtcmBufferMutex=nullptr;if(failure==1)deniedLock=rtcmBufferMutex;
        if(failure==2)gnssSerialMutex=nullptr;if(failure==3)deniedLock=gnssSerialMutex;if(failure==4)Serial1.shortWrite=true;
        assert(!agent.writeUnicoreCommand(reinterpret_cast<const uint8_t*>("mode rover\r\n"),12,error));assert(!error.isEmpty());
    }
    resetUart();assert(!agent.writeUnicoreCommand(nullptr,0,error));
    ascii("mode",false);assert(!agent.writeUnicoreCommand(reinterpret_cast<const uint8_t*>("mode rover\r\n"),12,error));
    resetUart();ascii("mode");const std::string survey="mode base time 120\r\n";
    assert(agent.writeUnicoreCommand(reinterpret_cast<const uint8_t*>(survey.data()),survey.size(),error)&&agent.surveyInMonitoring_&&agent.surveyInTargetSeconds_==120);
    resetUart();ascii("reset");assert(agent.writeUnicoreCommand(reinterpret_cast<const uint8_t*>("reset\r\n"),7,error)&&millis()==3000);
    agent.sensorType_="Ublox";assert(!agent.writeGnssCommands({}));
    resetUart();assert(agent.writeGnssCommands({{'$','D','E','L','A','Y','_','2','0','$'},{1,2,3}}));assert(millis()==1120);
    resetUart();Serial1.shortWrite=true;assert(!agent.writeGnssCommands({{1,2,3}}));
    for(unsigned failure=0;failure<5;++failure){
        resetUart();agent.sensorType_="Ublox";freshGga=failure!=1;
        const auto commands=GnssCmdBuilder::buildRoverCommand(GnssCmdBuilder::GnssSensorType::Ublox);
        for(const auto &command:commands)Serial1.replies.push_back(ubx(5,1,{command[2],command[3]}));
        std::vector<uint8_t> mode(40);mode[2]=failure==2?1:0;Serial1.replies.push_back(ubx(6,0x71,mode));
        if(failure==3)deniedLock=gnssSerialMutex;if(failure==4)Serial1.shortWrite=true;
        assert(agent.applyRoverMode()==(failure==0));assert(acquiredLocks==releasedLocks);
    }
    resetUart();agent.sensorType_="Unicorecomm";for(const char *name:{"mode","gpgga","gpgst","saveconfig"})ascii(name);
    assert(agent.applyRoverMode());resetUart();assert(!agent.writeGnssCommands({{'m','o','d','e',' ','r','o','v','e','r'}}));
}
'''
with tempfile.TemporaryDirectory(prefix='gnss-lifecycle-') as directory:
    path=Path(directory)
    (path/'check.cpp').write_text(harness+header+stubs+mapped(source_path,source,stripped_includes=True)+checks)
    compile_cpp(['c++','-std=c++17','-DNATIVE_BUILD','-fsanitize=address,undefined','-I'+str(root/'test/host/readback'),
                 '-I'+str(root/'test/host'),'-I'+str(root/'include'),'-I'+str(root/'.pio/libdeps/tdm240x_dev_general/ArduinoJson/src'),
                 str(path/'check.cpp'),str(root/'src/functions/backend/Backend_Agent_Codec.cpp'),str(root/'src/functions/gnss/gnss_cmd_builder.cpp'),
                 str(root/'src/functions/gnss/ublox_commands.cpp'),str(root/'src/functions/gnss/unicore_commands.cpp'),'-o',str(path/'check')],check=True)
    run_cpp([str(path/'check')],check=True)
print('PASS: complete GNSS verification/readback/polling, Survey-In persistence, ASCII ACK and UBX-confirmed Rover transitions')
