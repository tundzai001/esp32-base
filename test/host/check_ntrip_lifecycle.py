"""Run complete source/Rover transport modules with deterministic socket and UART I/O."""
from pathlib import Path
import re
import tempfile
from host_coverage import compile_cpp, run_cpp, mapped

root = Path(__file__).resolve().parents[2]
shim = (root/'test/host/Arduino.h').read_text().replace('#pragma once', '')
shim = '#define HOST_READBACK_TEST\n' + shim.replace('    void trim() {', '''
    String(unsigned long value, int) { char text[24]; snprintf(text, sizeof(text), "%lx", value); assign(text); }
    int indexOf(const char *value) const { auto p=find(value); return p==npos?-1:int(p); }
    void remove(size_t start, size_t count) { erase(start,count); }
    void replace(const String &from,const String &to) { auto p=find(from); if(p!=npos) std::string::replace(p,from.size(),to); }
    void trim() {''')
shim = shim.replace('    void println(const char *) {}', '    void println(const String &) {}')
harness = r'''
#include <ArduinoJson.h>
#include "functions/backend/Vn2000_Datum.h"
#include "functions/backend/Backend_Agent_Codec.h"
#include <array>
#include <cassert>
#include <cstring>
#include <deque>
#include <vector>
using std::min;
constexpr unsigned NTRIP_CASTER_PORT=2101,NTRIP_MODE=1,MUTEX_TIMEOUT_MS=1500,pdTRUE=1;
constexpr const char *NTRIP_CASTER_ADDRESS="caster",*NTRIP_MOUNTPOINT="mount",*NTRIP_AUTH_BASE_STATION="secret",*AGENT_VERSION="test";
constexpr int HEX=16;
using SemaphoreHandle_t=void*;using TinyGsm=int;
uint32_t now=10000;uint32_t millis(){return now;}void delay(unsigned n){now+=n;}
unsigned pdMS_TO_TICKS(unsigned n){return n;}
void *rtcmBufferMutex=(void*)1,*tcpStreamMutex=(void*)2;
bool denyLock=false;unsigned locks=0,unlocks=0;
bool xSemaphoreTake(void*,unsigned){if(denyLock)return false;++locks;return true;}
void xSemaphoreGive(void*){++unlocks;}
bool wifi=true;bool isWifiConnection(){return wifi;}
struct Client{
    bool up=false,accept=true,shortWrite=false;
    String response="HTTP/1.1 200 OK\r\n\r\n",wire;
    std::deque<uint8_t> input;
    unsigned stops=0,attempts=0;
    bool connected(){return up;}void stop(){++stops;up=false;input.clear();}
    void setTimeout(unsigned){}
    bool connect(const char*,unsigned){++attempts;up=accept;return up;}
    int available(){return input.size();}int read(){if(input.empty())return -1;int v=input.front();input.pop_front();return v;}
    int read(uint8_t *out,size_t n){size_t i=0;while(i<n&&!input.empty())out[i++]=read();return i;}
    size_t print(const String &text){wire+=text;if(text.find("HTTP/1.")!=String::npos)input.insert(input.end(),response.begin(),response.end());return text.size();}
    size_t write(const uint8_t *data,size_t n){if(shortWrite&&n)--n;wire.append((const char*)data,n);return n;}
};
using WiFiClient=Client;int modem=0;struct TinyGsmClient:Client{TinyGsmClient(int&,int){}};
struct Preferences{
    void begin(const char*,bool){}void end(){}
    String getString(const char*,const String &fallback){return fallback;}
    uint16_t getUShort(const char*,unsigned fallback){return fallback;}
}prefs;
struct BackendAgent{
    String service;bool workflow=false;
    String serviceConfigJson(){return service;}
    bool workflowActive(){return workflow;}
    static bool effectiveStreamActive(JsonObjectConst,unsigned,bool){return true;}
}backendAgent;
bool locked=false,licensed=true,provisioned=true,configured=true;
bool backendDeviceLocked(unsigned){return locked;}bool backendLicenseValid(){return licensed;}
bool backendDeviceProvisioned(){return provisioned;}bool backendBaseStartupConfigured(){return configured;}
bool backendControlPlaneFailOpen(){return false;}
struct GnssTelemetrySnapshot{std::array<char,128> lastGga={'$','G','N','G','G','A',',','t','e','s','t'};uint32_t lastGgaMs=10000;std::array<char,16> fixStatus={'R','T','K','_','F','I','X','E','D'};}snapshot;
auto getGnssTelemetrySnapshot(){return snapshot;}
struct {bool shortWrite=false;std::vector<uint8_t> data;size_t write(const uint8_t *p,size_t n){if(shortWrite&&n)--n;data.insert(data.end(),p,p+n);return n;}}Serial1;
// Public forward declaration normally supplied by ntrip_base.h.
bool ntripServerConnected(uint8_t);
'''
checks = r'''
int main(){
    assert(setupNTRIP()==0&&connectNTRIP()==0&&isNtripConnected());
    auto &client=clientFor(0);assert(&activeNtripClient()==&client);
    assert(client.wire.find("SOURCE secret /mount HTTP/1.1\r\n")!=String::npos);
    assert(client.wire.find("Authorization: Basic c291cmNlOnNlY3JldA==")!=String::npos);
    String data="abc",empty;
    assert(loopNTRIP(data)==5);now+=1000;assert(ntripServerBps(1)==3);
    assert(ntripServerError(0).isEmpty()&&ntripServerBps(3)==0&&!ntripServerConnected(0));
    stopNtrip();assert(!isNtripConnected());
    client.accept=false;assert(connectNTRIP()==-1&&ntripServerError(1)=="TCP connection failed");
    client.accept=true;client.response="HTTP/1.1 401 secret\r\n\r\n";
    assert(connectNTRIP()==-1&&ntripServerError(1).find("secret")==String::npos);
    client.response="HTTP/1.1 200 OK\r\n\r\n";
    backendAgent.service=R"({"server1_enabled":true,"serverhost1":"caster","mountpoint1":"m","ntrip_version1":2,"server2_enabled":true,"serverhost2":"other","mountpoint2":"n"})";
    assert(connectNTRIP()==0&&ntripServerConnected(2));
    assert(client.wire.find("POST /m HTTP/1.1\r\n")!=String::npos);
    assert(loopNTRIP(data)==5);now+=10000;assert(loopNTRIP(empty)==1);
    assert(client.wire.find("0\r\n\r\n")!=String::npos);
    client.shortWrite=true;clientFor(1).shortWrite=true;assert(loopNTRIP(data)==1);
    client.shortWrite=false;clientFor(1).shortWrite=false;
    locked=true;assert(loopNTRIP(data)==504);locked=false;
    for(bool *gate:{&licensed,&provisioned,&configured}){*gate=false;assert(connectNTRIP()==-1);*gate=true;}
    assert(connectNTRIP()==0);
    assert(!startNtripRover("host",0,"","","m",1));
    assert(!startNtripRover("host",2101,"","","///",1));
    assert(!startNtripRover("host\n",2101,"","","m",1));
    client.accept=false;assert(!startNtripRover("host",2101,"","pass","m",1));client.accept=true;
    client.response="HTTP/1.1 401 Unauthorized\r\n\r\n";
    assert(!startNtripRover("host",2101,"","pass","m",1));
    client.response="ICY 200 OK\r\n";client.response+=char(0xd3);client.response+="correction";
    assert(startNtripRover("host",2101,"","pass","/m",1)&&ntripRoverActive());
    assert(client.wire.find("GET /m HTTP/1.0\r\n")!=String::npos);
    assert(client.available()==11);assert(loopNTRIP(data)==504);
    denyLock=true;assert(loopNtripRover()==200&&ntripRoverBytesReceived()==0);denyLock=false;
    assert(loopNtripRover()==200&&ntripRoverBytesReceived()==11&&ntripRoverPacketsReceived()==1);
    assert(Serial1.data.front()==0xd3&&!ntripRoverDatum().has1021);
    client.input.insert(client.input.end(),504,0x12);
    assert(loopNtripRover()==200&&ntripRoverBytesReceived()==515);
    client.input.push_back(1);Serial1.shortWrite=true;assert(loopNtripRover()==504);
    assert(String(ntripRoverLastError())=="GNSS UART did not accept all CORS bytes");Serial1.shortWrite=false;
    now+=5000;assert(loopNtripRover()==200);
    client.up=false;assert(loopNtripRover()==504&&!ntripRoverActive());
    assert(loopNtripRover()==504);stopNtripRover();
    client.response="HTTP/1.1 200 OK\r\n\r\n";
    assert(startNtripRover("host",2101,"user","pass","m",2));
    assert(client.wire.find("GET /m HTTP/1.1\r\n")!=String::npos);
    assert(ntripRoverBytesReceived()==0&&ntripRoverPacketsReceived()==0);stopNtripRover();
    wifi=false;assert(&activeNtripClient()!=&client&&setupNTRIP()==0&&connectNTRIP()==0);
    assert(ntripServerConnected(1)&&ntripServerConnected(2));stopNtrip();
    assert(locks==unlocks);
}
'''
with tempfile.TemporaryDirectory(prefix='ntrip-lifecycle-') as directory:
    path=Path(directory)
    (path/'Arduino.h').write_text('#pragma once\n'+shim)
    agent_path=root/'src/functions/backend/Backend_Agent.cpp'
    agent=agent_path.read_text()
    helpers=agent[agent.index('bool jsonBool('):agent.index('void copyJsonObject(')]
    # The defaults are declared in the real internal header.
    declarations='namespace BackendAgentInternal { String jsonString(JsonObjectConst,const String&,const String &fallback=String()); }\n'
    body='#include <Arduino.h>\n'+harness+declarations+'namespace BackendAgentInternal {\n'+mapped(agent_path,helpers)+'}\n'
    header=(root/'include/functions/ntrip/ntrip_common.h').read_text()
    body+=re.sub(r'^#(?:include|pragma).*$', '',header,flags=re.M)
    for name in ('common','base','rover'):
        source_path=root/f'src/functions/ntrip/ntrip_{name}.cpp'
        source=re.sub(r'^#include.*$', '',source_path.read_text(),flags=re.M)
        body+=mapped(source_path,source,stripped_includes=True)
    (path/'check.cpp').write_text(body+checks)
    compile_cpp(['c++','-std=c++17','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-DNATIVE_BUILD',
                 '-I'+str(path),'-I'+str(root/'test/host'),'-I'+str(root/'include'),'-I'+str(root/'.pio/libdeps/tdm240x_dev_general/ArduinoJson/src'),
                 str(path/'check.cpp'),str(root/'src/functions/backend/Backend_Agent_Codec.cpp'),
                 str(root/'src/functions/backend/Backend_Agent_Projection.cpp'),'-o',str(path/'check')],check=True)
    run_cpp([str(path/'check')],check=True)
print('PASS: full NTRIP source/Rover lifecycle, auth and rejection, gates, transports, chunking, RTCM boundaries, UART short writes and lock balance')
