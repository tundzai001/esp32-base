"""Run complete workflow orchestration with controlled UART/CORS/RTOS boundaries."""
from pathlib import Path
import re
import tempfile
from host_coverage import compile_cpp, run_cpp, mapped

root = Path(__file__).resolve().parents[2]
source_path = root / 'src/functions/backend/Backend_Agent_Workflows.cpp'
source = re.sub(r'^#include.*$', '', source_path.read_text(), flags=re.M)
shim = (root / 'test/host/Arduino.h').read_text()
shim = shim.replace('    void trim() {', '''    long toInt() const { return std::strtol(c_str(), nullptr, 10); }
    void trim() {''')
harness = r'''
#include <Arduino.h>
#include <ArduinoJson.h>
#include "functions/gnss/gnss_cmd_builder.h"
#include "functions/backend/Base_Sample.h"
#include "functions/backend/Vn2000_Datum.h"
#include <cassert>
#include <array>
#include <cstring>
#include <limits>
#include <vector>
constexpr size_t CONFIG_DOCUMENT_SIZE=8192, COMMAND_DOCUMENT_SIZE=8192;
constexpr unsigned MUTEX_TIMEOUT_MS=1500, pdTRUE=1, pdPASS=1;
using BaseType_t=int;
using SemaphoreHandle_t=void*;
uint32_t now=1000;
uint32_t millis(){return now;}
unsigned pdMS_TO_TICKS(unsigned n){return n;}
template<class T,class L,class H> T constrain(T value,L low,H high){return value<low?T(low):value>high?T(high):value;}
void *tcpStreamMutex=reinterpret_cast<void*>(1);
bool locked=false, createTaskOk=true, roverOk=true, startOk=true, restoreOk=true, deliverLocation=true;
bool stopOk=true;
std::string fault;
unsigned locks=0,unlocks=0,stops=0,starts=0,saves=0,taskDeletes=0,receiverWrites=0;
BaseType_t xTaskCreatePinnedToCore(void(*)(void*),const char*,unsigned,void*,unsigned,void **task,unsigned){
    if(createTaskOk)*task=reinterpret_cast<void*>(1);return createTaskOk?pdPASS:0;
}
void vTaskDelete(void*){++taskDeletes;}
unsigned xSemaphoreTake(void*,unsigned){if(locked)return 0;++locks;return pdTRUE;}
void xSemaphoreGive(void*){++unlocks;}
void stopNtrip(){++stops;}
void stopNtripRover(){++stops;}
void setupNTRIP(){++starts;}
void connectNTRIP(){}
bool startNtripRover(const String&,uint16_t,const String&,const String&,const String&,uint8_t){return startOk;}
const char *ntripRoverLastError(){return "injected CORS disconnect";}
Vn2000Datum sessionDatum;
Vn2000Datum ntripRoverDatum(){return sessionDatum;}
namespace BackendAgentInternal {
String epochString(){return "123";}
bool jsonBool(JsonObjectConst object,const String &key,bool fallback){return object[key].isNull()?fallback:object[key].as<bool>();}
}
struct GnssTelemetrySnapshot {uint32_t lastGgaMs; std::array<char,16> fixStatus{}; double latitude,longitude,altitude;};
struct BackendAgent {
    struct AutoBaseLocationReply{char json[1536];};
    void *workflowTask_=nullptr,*autoBaseLocationQueue_=reinterpret_cast<void*>(1);
    bool workflowCancelRequested_=false,surveyInResultPending_=false;
    uint32_t surveyInTargetSeconds_=120;
    String workflowKind_,workflowPayload_,autoBaseProgress_,agentState_,statusBootId_="boot-1";
    String baseConfig_=R"({"base_setup_method":"FIXED_LLA","coords":{"lat":21,"lon":105,"alt":12}})";
    String sensorType_="Ublox",baseStartupState_,baseStartupError_,receiverBaseReadback_;
    std::vector<String> phases;
    void setWorkflowProgress(const String&,uint8_t,uint8_t,const String&=String());
    bool startWorkflow(const String&,const String&); void requestWorkflowStop();
    static void workflowTaskThunk(void*); bool restartConfiguredNtrip();int loopNtripRoverLocked();
    bool stopNtripRoverLocked();bool applyBaseCoordinates(double,double,double);
    bool restoreWorkflowBase(JsonObjectConst);void runWorkflow();
    bool runAutoBaseWorkflow(JsonObject);bool runReferenceCheckWorkflow(JsonObject);
    bool waitForAutoBaseFix(uint32_t, uint8_t, bool &, String &);
    bool collectAutoBaseSamples(int, std::vector<double> &, std::vector<double> &, std::vector<double> &, bool &, String &);
    void setAgentState(const String &value){agentState_=value;}
    String normalAgentState()const{return "online";}
    void markStatusDirty(){
        DynamicJsonDocument progress(8192);deserializeJson(progress,autoBaseProgress_);
        phases.push_back(progress["phase"]|String());
    }
    bool saveStatePreferences(){++saves;return true;}
    bool applyRoverMode(){return roverOk;}
    bool writeGnssCommands(const GnssCmdBuilder::CommandList &commands){++receiverWrites;return restoreOk&&!commands.empty();}
    bool refreshReceiverReadback(String&){receiverBaseReadback_=R"({"mode_kind":"base"})";return restoreOk;}
} *current=nullptr;
void vTaskDelay(unsigned ticks){
    now+=ticks;
    if(current&&((fault=="cancel_wait"&&current->agentState_=="auto_setup_wait_fix")||
                 (fault=="cancel_average"&&current->agentState_=="auto_setup_averaging")))current->workflowCancelRequested_=true;
}
bool averaging(){return current->agentState_=="auto_setup_averaging"||current->agentState_=="reference_check_averaging";}
int loopNtripRover(){return (fault=="lose_wait"&&!averaging())||(fault=="lose_average"&&averaging())?504:200;}
GnssTelemetrySnapshot getGnssTelemetrySnapshot(){
    bool fixed=fault!="no_fix"&&!(fault=="no_samples"&&averaging());
    GnssTelemetrySnapshot snapshot{};snapshot.lastGgaMs=now;
    std::strcpy(snapshot.fixStatus.data(),fixed?"RTK_FIXED":"SINGLE");
    snapshot.latitude=21.0;snapshot.longitude=105.0;snapshot.altitude=12.0;return snapshot;
}
void xQueueReset(void*){}
unsigned xQueueReceive(void*,void *target,unsigned ticks){
    now+=ticks;
    if(!deliverLocation)return 0;
    DynamicJsonDocument progress(8192),reply(2048);
    assert(deserializeJson(progress,current->autoBaseProgress_)==DeserializationError::Ok);
    JsonObject request=progress["location_request"];
    reply["request_id"]=request["request_id"];
    reply["location"]["input_coord"]["lat"]=request["lat"];
    reply["location"]["input_coord"]["lon"]=request["lon"];
    reply["location"]["input_coord"]["alt"]=request["alt"];
    reply["location"]["l0_deg"]=105;reply["location"]["k0"]=0.9996;
    reply["location"]["province_code"]="01";
    serializeJson(reply,static_cast<BackendAgent::AutoBaseLocationReply*>(target)->json,1536);
    return pdTRUE;
}
'''.replace('const String&=String()', 'const String &details=String()')
checks = r'''
void reset(BackendAgent &agent){
    assert(locks==unlocks);current=&agent;now=1000;fault.clear();
    locked=false;createTaskOk=roverOk=startOk=restoreOk=deliverLocation=stopOk=true;
    tcpStreamMutex=reinterpret_cast<void*>(1);receiverWrites=saves=0;sessionDatum=Vn2000Datum();
    sessionDatum.has1021=sessionDatum.has1025=true;sessionDatum.centralMeridianDeg=105;sessionDatum.scale=0.9996;
}
JsonObject config(DynamicJsonDocument &doc){
    assert(deserializeJson(doc,R"({"ip":"caster.example","port":2101,"user":"test","password":"test","mountpoint":"BASE","timeout":30,"fixed_streak_seconds":1,"samples":5,"enable_itrf_vn2000_transform":false,"reference_coords":{"lat":21,"lon":105,"alt":12}})")==DeserializationError::Ok);
    return doc.as<JsonObject>();
}
String phase(BackendAgent &agent){DynamicJsonDocument doc(8192);deserializeJson(doc,agent.autoBaseProgress_);return doc["phase"]|String();}
int main(){
    for(unsigned mode=0;mode<4;++mode){
        BackendAgent agent;reset(agent);DynamicJsonDocument doc(8192);auto payload=config(doc);
        if(mode){payload["enable_itrf_vn2000_transform"]=true;sessionDatum.has1025=mode==1;}
        if(mode==2){payload["l0_deg"]=105;payload["k0"]=0.9999;}
        assert(agent.runAutoBaseWorkflow(payload)&&phase(agent)=="ready_for_review");
        DynamicJsonDocument progress(8192);deserializeJson(progress,agent.autoBaseProgress_);
        assert(progress["sample_count"]==60&&progress["receiver_write_completed"]==false);
        assert(progress["conversion_report"]["projection_source"]==(mode==0?"not_used":mode==1?"rtcm1025":mode==2?"manual":"server_catalog"));
        assert(receiverWrites==1&&agent.baseConfig_==R"({"base_setup_method":"FIXED_LLA","coords":{"lat":21,"lon":105,"alt":12}})");
        assert(saves==1&&locks==unlocks);
    }
    for(const char *failure:{"missing","rover","start","no_fix","no_samples","cancel_wait","cancel_average","lose_wait","lose_average","datum","lookup","shift","conversion","restore","lock"}){
        BackendAgent agent;reset(agent);DynamicJsonDocument doc(8192);auto payload=config(doc);fault=failure;
        if(fault=="missing")payload["ip"]="";
        if(fault=="rover")roverOk=false;
        if(fault=="start")startOk=false;
        if(fault=="cancel_wait")payload["fixed_streak_seconds"]=5;
        if(fault=="datum"){payload["enable_itrf_vn2000_transform"]=true;sessionDatum.has1021=false;}
        if(fault=="lookup"){payload["enable_itrf_vn2000_transform"]=true;sessionDatum.has1025=false;deliverLocation=false;}
        if(fault=="shift"){payload["enable_itrf_vn2000_transform"]=true;payload["resN"]="bad";}
        if(fault=="conversion"){payload["enable_itrf_vn2000_transform"]=true;sessionDatum.scale=0;}
        if(fault=="restore")restoreOk=false;
        if(fault=="lock")locked=true;
        assert(!agent.runAutoBaseWorkflow(payload));
        assert(phase(agent)==(fault=="cancel_wait"||fault=="cancel_average"?"stopped":"failed"));
        assert(saves>=1&&locks==unlocks&&agent.agentState_=="online");
    }
    for(const char *failure:{"success","missing","rover","start","no_fix","no_samples","lose_wait","lose_average","restore","lock"}){
        BackendAgent agent;reset(agent);DynamicJsonDocument doc(8192);auto payload=config(doc);fault=failure;
        if(fault=="missing")payload["reference_coords"]["lat"]=91;
        if(fault=="rover")roverOk=false;
        if(fault=="start")startOk=false;
        if(fault=="restore")restoreOk=false;
        if(fault=="lock")locked=true;
        assert(agent.runReferenceCheckWorkflow(payload)==(fault=="success"));
        assert(phase(agent)==(fault=="success"?"reference_check_completed":"reference_check_failed"));
        if(fault=="success"){
            DynamicJsonDocument progress(8192);deserializeJson(progress,agent.autoBaseProgress_);
            assert(progress["sample_count"]==5&&progress["delta_n_m"]==0&&progress["delta_e_m"]==0&&progress["delta_u_m"]==0);
        }
        assert(locks==unlocks);
    }
    BackendAgent task;reset(task);
    task.workflowPayload_="invalid";task.runWorkflow();assert(phase(task)=="failed");
    task.workflowPayload_="{}";task.workflowKind_="unknown";task.runWorkflow();assert(phase(task)=="failed");
    createTaskOk=false;assert(!task.startWorkflow("auto_base","{}")&&task.workflowTask_==nullptr);
    createTaskOk=true;assert(task.startWorkflow("auto_base","{}"));assert(!task.startWorkflow("reference_check","{}"));
    task.requestWorkflowStop();assert(task.workflowCancelRequested_);
    BackendAgent::workflowTaskThunk(&task);assert(task.workflowTask_==nullptr&&!task.workflowCancelRequested_&&task.workflowPayload_.isEmpty());
    BackendAgent::workflowTaskThunk(nullptr);assert(taskDeletes==2);
}
'''
with tempfile.TemporaryDirectory(prefix='workflow-lifecycle-') as directory:
    path = Path(directory)
    (path / 'Arduino.h').write_text(shim)
    (path / 'check.cpp').write_text(harness + mapped(source_path, source, stripped_includes=True) + checks)
    compile_cpp(['c++', '-std=c++17', '-fsanitize=address,undefined', '-I'+str(path), '-I'+str(root/'test/host'), '-I'+str(root/'include'),
                 '-I'+str(root/'.pio/libdeps/tdm240x_dev_general/ArduinoJson/src'), str(path/'check.cpp'),
                 str(root/'src/functions/backend/Backend_Agent_Projection.cpp'),
                 str(root/'src/functions/gnss/gnss_cmd_builder.cpp'),str(root/'src/functions/gnss/ublox_commands.cpp'),
                 str(root/'src/functions/gnss/unicore_commands.cpp'),'-o',str(path/'check')],check=True)
    run_cpp([str(path/'check')],check=True)
print('PASS: complete Auto Base/reference lifecycle, real 60-sample averaging, review report, location fallback, cleanup, cancellation and disconnects')
